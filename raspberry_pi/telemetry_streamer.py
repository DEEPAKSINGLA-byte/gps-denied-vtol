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


def get_default_gateway_or_client_ip() -> Optional[str]:
    """
    Attempt to discover the client IP or laptop IP automatically.
    """
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
        stream_mode: str = "both",
        rate_hz: float = 10.0,
        pixhawk_port: Optional[str] = None,
        pixhawk_baud: int = 115200,
    ):
        self.target_ip = target_ip
        self.mavlink_port = mavlink_port
        self.json_port = json_port
        self.stream_mode = stream_mode.lower()
        self.rate_hz = rate_hz
        self.interval = 1.0 / max(rate_hz, 1.0)
        self.pixhawk_port = pixhawk_port
        self.pixhawk_baud = pixhawk_baud

        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
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
        print(f"Stream Rate:         {self.rate_hz:.1f} Hz")
        print("=" * 68)

        # Connect to Pixhawk
        print(f"\n[1/2] Connecting to Pixhawk flight controller...")
        self.vehicle = PixhawkVehicle(
            connection_str=self.pixhawk_port,
            baud=self.pixhawk_baud
        )

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
        if self.vehicle:
            self.vehicle.disconnect()
        if self.udp_sock:
            self.udp_sock.close()
        print("\nTelemetry streamer stopped.")

    def _json_stream_worker(self):
        """
        Periodically serialize full sensor snapshot and send as UDP datagram.
        """
        json_target = (self.target_ip, self.json_port)
        seq = 0
        while self.running:
            try:
                telem = self.vehicle.get_telemetry()
                telem["seq"] = seq
                payload = json.dumps(telem).encode("utf-8")
                self.udp_sock.sendto(payload, json_target)
                self.packets_sent_json += 1
                seq += 1
            except Exception as e:
                pass
            time.sleep(self.interval)




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
    parser.add_argument(
        "--mode",
        type=str,
        choices=["both", "json", "mavlink"],
        default="both",
        help="Streaming mode: 'both', 'json', or 'mavlink' (default: both)"
    )
    parser.add_argument("--rate", type=float, default=10.0, help="JSON telemetry rate in Hz (default: 10.0)")
    parser.add_argument("--port", type=str, default=None, help="Pixhawk serial port (default: auto-detect)")
    parser.add_argument("--baud", type=int, default=115200, help="Pixhawk serial baud (default: 115200)")

    args = parser.parse_args()

    streamer = TelemetryStreamer(
        target_ip=args.target_ip,
        mavlink_port=args.mavlink_port,
        json_port=args.json_port,
        stream_mode=args.mode,
        rate_hz=args.rate,
        pixhawk_port=args.port,
        pixhawk_baud=args.baud,
    )
    streamer.start()


if __name__ == "__main__":
    main()
