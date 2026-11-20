#!/usr/bin/env python3
"""
telemetry_streamer.py - Stream Pixhawk Telemetry from Raspberry Pi to Laptop
============================================================================
Reads real-time flight telemetry and sensor data from Pixhawk over USB/UART
and transmits it over the wireless network (e.g., Redmi 10) to the laptop.

Supports dual streaming protocols:
  1. Standard MAVLink UDP (port 14550): Compatible with QGroundControl, Mission Planner, and pymavlink.
  2. Structured JSON UDP (port 14551): Lightweight, easily parseable sensor frames for custom scripts.

Usage Examples:
  # Stream to specific laptop IP (over Redmi 10 network)
  python3 telemetry_streamer.py --target-ip 10.184.3.231

  # Stream at 10 Hz with custom ports
  python3 telemetry_streamer.py --target-ip 10.184.3.231 --rate 10

  # Stream JSON only or MAVLink only
  python3 telemetry_streamer.py --target-ip 10.184.3.231 --mode json
  python3 telemetry_streamer.py --target-ip 10.184.3.231 --mode mavlink

Latency design (JSON stream, the one the ROS bridge uses):
  * Event-driven: a packet goes out as soon as fresh navigation data (ATTITUDE,
    LOCAL_POSITION_NED, OPTICAL_FLOW, RAW_IMU, DISTANCE_SENSOR) arrives from the Pixhawk,
    after a 3 ms coalescing window so one autopilot burst becomes one packet. --rate is
    the MAX rate; a keepalive still goes out every 200 ms if no nav data arrives.
  * The Pixhawk is asked for those messages at --fast-rate Hz (default 30) instead of the
    10 Hz REQUEST_DATA_STREAM groups.
  * Every packet fits in ONE UDP datagram (<= 1400 B). The full snapshot is ~1.6-1.8 kB,
    which IP-fragments on Wi-Fi (lose either fragment -> lose the packet). Fields no
    ground consumer needs every cycle (global_position, ekf_status, vibration,
    rc_channels, statustexts) are rotated across packets instead.
  * DSCP CS6 -> Wi-Fi WMM voice queue (ahead of the camera's bulk traffic).
"""

import sys
import os
import time
import json
import socket
import argparse
import threading
from typing import Optional
from pixhawk_controller import PixhawkVehicle
from pymavlink import mavutil

# Largest UDP payload that never IP-fragments on a 1500-MTU link is 1472 B; keep margin
# for VPN/PPPoE-style overheads.
MAX_DATAGRAM = 1400
# Bulky fields that no ground consumer reads every cycle: rotated across packets.
SLOW_KEYS = ("global_position", "ekf_status", "vibration", "rc_channels", "statustexts")
DSCP_CS6 = 0xC0          # WMM voice access category
NAV_COALESCE_S = 0.003   # wait for the rest of the autopilot's message burst
KEEPALIVE_S = 0.2        # send at least this often even without fresh nav data


def _dumps(obj) -> bytes:
    return json.dumps(obj, separators=(",", ":")).encode("utf-8")


def pack_telemetry(telem: dict, cursor: int, max_bytes: int = MAX_DATAGRAM):
    """Encode one telemetry snapshot as a single datagram of at most ``max_bytes``.

    All non-SLOW_KEYS fields are always included. SLOW_KEYS are added round-robin starting
    at ``cursor`` while they fit, so each one is still delivered every few packets.
    Returns (payload_bytes, next_cursor).
    """
    pkt = {k: v for k, v in telem.items() if k not in SLOW_KEYS}
    base = _dumps(pkt)
    if len(base) > max_bytes:          # should not happen; never send a fragmenting packet
        return base, cursor            # (logged by the caller via size check)
    size = len(base)
    n = len(SLOW_KEYS)
    added = 0
    for i in range(n):
        k = SLOW_KEYS[(cursor + i) % n]
        if k not in telem:
            added += 1
            continue
        val = telem[k]
        extra = len(_dumps({k: val})) - 1   # '{"k":v}' -> ',"k":v' adds len-1 bytes
        if size + extra > max_bytes and isinstance(val, list) and len(val) > 1:
            # e.g. a burst of STATUSTEXTs: send the newest entries that fit
            while len(val) > 1 and size + extra > max_bytes:
                val = val[1:]
                extra = len(_dumps({k: val})) - 1
        if size + extra > max_bytes:
            break
        pkt[k] = val
        size += extra
        added += 1
    return _dumps(pkt), (cursor + max(added, 1)) % n


def get_default_gateway_or_client_ip() -> Optional[str]:
    """
    Attempt to discover the client IP or laptop IP automatically.
    """
    # Explicit override
    env_ip = os.environ.get("GPS_DENIED_LAPTOP_IP")
    if env_ip:
        return env_ip

    # Check SSH client environment variable
    ssh_client = os.environ.get("SSH_CLIENT")
    if ssh_client:
        client_ip = ssh_client.split()[0]
        return client_ip

    # Check routing table for default gateway
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
        s.close()
        # Fallback to common subnet
        prefix = local_ip.rsplit(".", 1)[0]
        return f"{prefix}.231"
    except Exception:
        return "10.184.3.231"


class TelemetryStreamer:
    def __init__(
        self,
        target_ip: str,
        mavlink_port: int = 14550,
        json_port: int = 14551,
        cmd_port: int = 14552,
        stream_mode: str = "both",
        rate_hz: float = 10.0,
        pixhawk_port: Optional[str] = None,
        pixhawk_baud: int = 115200,
        velocity_port: int = 0,
        fast_rate_hz: float = PixhawkVehicle.FAST_RATE_HZ,
    ):
        self.target_ip = target_ip
        self.mavlink_port = mavlink_port
        self.json_port = json_port
        self.cmd_port = cmd_port
        self.stream_mode = stream_mode.lower()
        self.rate_hz = rate_hz
        self.interval = 1.0 / max(rate_hz, 1.0)
        self.pixhawk_port = pixhawk_port
        self.pixhawk_baud = pixhawk_baud
        self.velocity_port = velocity_port  # >0 enables Nav2 velocity listener (shares this vehicle)
        self.fast_rate_hz = fast_rate_hz
        self.nav_listener = None

        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.udp_sock.setsockopt(socket.IPPROTO_IP, socket.IP_TOS, DSCP_CS6)
        except OSError:
            pass
        self.running = False
        self.vehicle: Optional[PixhawkVehicle] = None

        # Statistics
        self.packets_sent_mavlink = 0
        self.packets_sent_json = 0
        self.start_time = 0.0

    def start(self):
        print("=" * 68)
        print(" PIXHAWK -> LAPTOP WIRELESS TELEMETRY STREAMER")
        print("=" * 68)
        print(f"Target Laptop IP:    {self.target_ip}")
        print(f"Streaming Mode:      {self.stream_mode.upper()}")
        if self.stream_mode in ["both", "mavlink"]:
            print(f"  * MAVLink UDP Port: {self.target_ip}:{self.mavlink_port}")
        if self.stream_mode in ["both", "json"]:
            print(f"  * JSON UDP Port:    {self.target_ip}:{self.json_port}")
        print(f"Stream Rate:         event-driven, max {self.rate_hz:.1f} Hz "
              f"(Pixhawk nav msgs @ {self.fast_rate_hz:.0f} Hz)")
        print("=" * 68)

        # Connect to Pixhawk
        print(f"\n[1/2] Connecting to Pixhawk flight controller...")
        self.vehicle = PixhawkVehicle(
            connection_str=self.pixhawk_port,
            baud=self.pixhawk_baud,
            auto_connect=False,
        )
        self.vehicle.fast_rate_hz = self.fast_rate_hz   # must be set before connect()
        self.vehicle.connect()

        if not self.vehicle.is_connected:
            print("[ERROR] Could not connect to Pixhawk flight controller.", file=sys.stderr)
            sys.exit(1)

        print(f"[+] Connected to Pixhawk! Autopilot: {self.vehicle.autopilot_type} | Frame: {self.vehicle.vehicle_type}")
        print(f"[2/2] Initializing telemetry transmission to {self.target_ip}...\n")

        self.running = True
        self.start_time = time.time()

        # Register raw MAVLink forwarder callback (prevents thread contention on serial port)
        if self.stream_mode in ["both", "mavlink"]:
            mavlink_target = (self.target_ip, self.mavlink_port)
            def forward_mav(buf):
                try:
                    self.udp_sock.sendto(buf, mavlink_target)
                    self.packets_sent_mavlink += 1
                except Exception:
                    pass
            self.vehicle.raw_packet_callback = forward_mav

        threads = []
        if self.stream_mode in ["both", "json"]:
            t_json = threading.Thread(target=self._json_stream_worker, daemon=True, name="JSONStreamer")
            threads.append(t_json)
            t_json.start()

        # Start interactive ground station command listener (port 14552)
        t_cmd = threading.Thread(target=self._command_listener_worker, daemon=True, name="CommandListener")
        threads.append(t_cmd)
        t_cmd.start()

        if self.velocity_port > 0:
            from mavlink_cmd_listener import MavlinkCmdListener
            self.nav_listener = MavlinkCmdListener(
                self.pixhawk_port, self.pixhawk_baud, self.velocity_port, vehicle=self.vehicle)
            threading.Thread(target=self.nav_listener.run, daemon=True, name="NavCmdListener").start()
            print(f"[+] Nav2 velocity listener active on UDP 0.0.0.0:{self.velocity_port} (shared Pixhawk link)")

        try:
            while self.running:
                elapsed = time.time() - self.start_time
                if elapsed > 0:
                    mav_rate = self.packets_sent_mavlink / elapsed
                    json_rate = self.packets_sent_json / elapsed
                    telem = self.vehicle.get_telemetry()
                    att = telem.get("attitude") or {}
                    hud = telem.get("vfr_hud") or {}
                    baro = telem.get("barometer") or {}
                    bat = telem.get("battery") or {}

                    roll = att.get("roll_deg", 0.0)
                    pitch = att.get("pitch_deg", 0.0)
                    yaw = att.get("yaw_deg", 0.0)
                    alt = hud.get("alt_m", 0.0)
                    volt = bat.get("voltage_v", 0.0)
                    mode = telem.get("flight_mode", "UNKNOWN")

                    status_line = (
                        f"\r[STREAMING -> {self.target_ip}] "
                        f"Mode:{mode:<8} | Roll:{roll:>5.1f}° Pitch:{pitch:>5.1f}° Yaw:{yaw:>6.1f}° | "
                        f"Alt:{alt:>4.1f}m | Bat:{volt:>4.1f}V | "
                        f"JSON: {json_rate:>4.1f} pkt/s | MAV: {mav_rate:>4.1f} pkt/s"
                    )
                    sys.stdout.write(status_line)
                    sys.stdout.flush()
                time.sleep(0.5)
        except KeyboardInterrupt:
            print("\n\nStopping telemetry streamer...")
        finally:
            self.stop()

    def stop(self):
        self.running = False
        if self.nav_listener:
            self.nav_listener.stop()
        if self.vehicle:
            self.vehicle.disconnect()
        if self.udp_sock:
            self.udp_sock.close()
        print("\nTelemetry streamer stopped.")

    def _json_stream_worker(self):
        """
        Send a telemetry snapshot as soon as fresh nav data arrives (event-driven),
        at most rate_hz, and at least every KEEPALIVE_S.
        """
        json_target = (self.target_ip, self.json_port)
        nav_event = self.vehicle.nav_data_event
        seq = 0
        cursor = 0
        last_send = 0.0
        warned_size = False
        min_gap = 0.8 * self.interval   # slack so a source AT the cap rate is never throttled
        while self.running:
            fresh = nav_event.wait(KEEPALIVE_S)
            if fresh:
                time.sleep(NAV_COALESCE_S)          # let the rest of the burst land
            nav_event.clear()
            # Rate cap WITHOUT delaying data: sleeping here would hold a fresh burst back
            # (measured: +15 ms median). Too soon -> skip; the next burst triggers a send.
            if fresh and time.monotonic() - last_send < min_gap:
                continue
            try:
                telem = self.vehicle.get_telemetry()
                telem["seq"] = seq
                payload, cursor = pack_telemetry(telem, cursor)
                if len(payload) > MAX_DATAGRAM and not warned_size:
                    print(f"\n[!] Telemetry packet is {len(payload)} B (> {MAX_DATAGRAM}); "
                          "it will be IP-fragmented on Wi-Fi")
                    warned_size = True
                self.udp_sock.sendto(payload, json_target)
                self.packets_sent_json += 1
                seq += 1
            except Exception:
                pass
            last_send = time.monotonic()

    def _command_listener_worker(self):
        """
        Listen for incoming control commands (ARM, DISARM, SET_MODE, EMERGENCY_KILL)
        from the laptop Ground Station GUI and execute them directly via PixhawkVehicle.
        """
        cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            cmd_sock.bind(("0.0.0.0", self.cmd_port))
            print(f"[+] Ground Station command listener active on UDP 0.0.0.0:{self.cmd_port}")
        except Exception as e:
            print(f"[!] Warning: Could not bind command port {self.cmd_port}: {e}")
            return

        cmd_sock.settimeout(1.0)
        while self.running:
            try:
                data, addr = cmd_sock.recvfrom(4096)
                if not data or not self.vehicle:
                    continue
                cmd_data = json.loads(data.decode("utf-8"))
                action = str(cmd_data.get("action", "")).lower()
                resp = {"status": "error", "message": f"Unknown action '{action}'", "action": action}

                if action == "arm":
                    force = cmd_data.get("force", False)
                    ok, msg = self.vehicle.arm(force=force)
                    resp = {"status": "ok" if ok else "fail", "message": msg, "action": action}
                elif action == "disarm":
                    force = cmd_data.get("force", False)
                    ok, msg = self.vehicle.disarm(force=force)
                    resp = {"status": "ok" if ok else "fail", "message": msg, "action": action}
                elif action == "emergency_kill":
                    ok, msg = self.vehicle.emergency_kill()
                    resp = {"status": "ok" if ok else "fail", "message": msg, "action": action}
                elif action == "set_mode":
                    mode = cmd_data.get("mode", "")
                    ok, msg = self.vehicle.set_mode(mode)
                    resp = {"status": "ok" if ok else "fail", "message": msg, "action": action}
                elif action == "set_param":
                    name = cmd_data.get("name", "")
                    val = cmd_data.get("value", 0)
                    ok, msg = self.vehicle.set_parameter(name, val)
                    resp = {"status": "ok" if ok else "fail", "message": msg, "action": action}
                elif action == "ping":
                    resp = {"status": "ok", "message": "pong", "time": time.time(), "action": action}

                cmd_sock.sendto(json.dumps(resp).encode("utf-8"), addr)
            except socket.timeout:
                continue
            except Exception as e:
                pass
        cmd_sock.close()




def main():
    default_ip = get_default_gateway_or_client_ip()
    parser = argparse.ArgumentParser(description="Stream Pixhawk Telemetry to Laptop over Wi-Fi")
    parser.add_argument(
        "--target-ip",
        type=str,
        default=default_ip,
        help=f"Target laptop IP address (default: {default_ip})"
    )
    parser.add_argument("--mavlink-port", type=int, default=14550, help="Target MAVLink UDP port (default: 14550)")
    parser.add_argument("--json-port", type=int, default=14551, help="Target JSON UDP port (default: 14551)")
    parser.add_argument("--cmd-port", type=int, default=14552, help="Command listen UDP port (default: 14552)")
    parser.add_argument(
        "--mode",
        type=str,
        choices=["both", "json", "mavlink"],
        default="both",
        help="Streaming mode: 'both', 'json', or 'mavlink' (default: both)"
    )
    parser.add_argument("--velocity-port", type=int, default=0,
                        help="Enable Nav2 velocity listener on this UDP port (e.g. 14553; 0 = off). Forwards only while armed in GUIDED.")
    parser.add_argument("--rate", type=float, default=30.0,
                        help="MAX JSON telemetry rate in Hz; packets are sent when fresh nav data "
                             "arrives (default: 30)")
    parser.add_argument("--fast-rate", type=float, default=PixhawkVehicle.FAST_RATE_HZ,
                        help="Rate (Hz) requested from the Pixhawk for ATTITUDE, LOCAL_POSITION_NED, "
                             "OPTICAL_FLOW, RAW_IMU, DISTANCE_SENSOR; 0 = leave autopilot defaults "
                             f"(default: {PixhawkVehicle.FAST_RATE_HZ:.0f})")
    parser.add_argument("--port", type=str, default=None, help="Pixhawk serial port (default: auto-detect)")
    parser.add_argument("--baud", type=int, default=115200, help="Pixhawk serial baud (default: 115200)")

    args = parser.parse_args()

    streamer = TelemetryStreamer(
        target_ip=args.target_ip,
        mavlink_port=args.mavlink_port,
        json_port=args.json_port,
        cmd_port=args.cmd_port,
        stream_mode=args.mode,
        rate_hz=args.rate,
        pixhawk_port=args.port,
        pixhawk_baud=args.baud,
        velocity_port=args.velocity_port,
        fast_rate_hz=args.fast_rate,
    )
    streamer.start()


if __name__ == "__main__":
    main()
