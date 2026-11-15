#!/usr/bin/env python3
"""
mavlink_cmd_listener.py - Pi-side listener for ROS 2 Nav2 velocity commands

Runs on the Raspberry Pi companion computer. Listens on UDP port 14553 for
JSON velocity setpoints from the laptop's mavlink_cmd_bridge node and forwards
them to the Pixhawk flight controller via MAVLink using PixhawkVehicle.

Protocol:
  Incoming JSON (from laptop mavlink_cmd_bridge on UDP 14553):
    {"action": "velocity", "vx": 0.5, "vy": 0.0, "vz": 0.0,
     "yawspeed": 0.0, "frame": "body", "timestamp": ...}

  Coordinate convention (body frame):
    vx = forward (+X), vy = left (+Y), vz = up (+Z), yawspeed = yaw rate (rad/s, CCW +)
    These are converted to NED and sent as SET_POSITION_TARGET_LOCAL_NED (GUIDED).

Usage:
  python3 mavlink_cmd_listener.py --port /dev/pixhawk --baud 115200 --listen-port 14553
"""

import argparse
import json
import math
import socket
import sys
import threading
import time

from pixhawk_controller import PixhawkVehicle


class MavlinkCmdListener:
    def __init__(self, connection_str: str, baud: int, listen_port: int,
                 vehicle: PixhawkVehicle | None = None):
        """If ``vehicle`` is given (e.g. from telemetry_streamer.py) it is shared
        and left connected on shutdown, so only ONE process owns the serial port."""
        self.connection_str = connection_str
        self.baud = baud
        self.listen_port = listen_port

        self.vehicle: PixhawkVehicle | None = vehicle
        self._owns_vehicle = vehicle is None
        self.connected = vehicle is not None and vehicle.is_connected

        # Latest velocity command
        self.lock = threading.Lock()
        self.last_cmd = None
        self.last_cmd_time = 0.0

        # UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('0.0.0.0', listen_port))
        self.sock.setblocking(False)

        self._stop_event = threading.Event()
        self._udp_thread = threading.Thread(
            target=self._udp_receive_loop, daemon=True, name='MavlinkCmdUDP'
        )
        self._udp_thread.start()

        # Connect to Pixhawk (unless a shared, already-connected vehicle was supplied)
        if self._owns_vehicle:
            self._connect()

    def _connect(self):
        try:
            self.vehicle = PixhawkVehicle(
                connection_str=self.connection_str,
                baud=self.baud,
                timeout=10.0,
                auto_connect=True,
            )
            if self.vehicle.is_connected:
                self.connected = True
                print(f'[LISTENER] Connected to Pixhawk on {self.connection_str}')
            else:
                print('[LISTENER] Failed to connect to Pixhawk.')
                self.connected = False
        except Exception as e:
            print(f'[LISTENER] Connection error: {e}')
            self.connected = False

    def _udp_receive_loop(self):
        while not self._stop_event.is_set():
            try:
                data, _ = self.sock.recvfrom(65535)
            except BlockingIOError:
                time.sleep(0.002)
                continue
            except Exception:
                time.sleep(0.01)
                continue

            try:
                cmd = json.loads(data.decode('utf-8'))
            except Exception:
                continue

            with self.lock:
                self.last_cmd = cmd
                self.last_cmd_time = time.time()

    def _send_velocity_setpoint(self, vx: float, vy: float, vz: float, yawspeed: float):
        """Send a velocity setpoint to ArduPilot (GUIDED) via SET_POSITION_TARGET_LOCAL_NED.

        Input is the ROS body frame (REP-103, FLU): vx forward, vy LEFT, vz UP,
        yawspeed counter-clockwise-positive (rad/s).
        Output is MAV_FRAME_LOCAL_NED: north/east/down velocities and a clockwise-
        positive yaw rate, so the conversion is:
            north =  cos(yaw)*vx + sin(yaw)*vy
            east  =  sin(yaw)*vx - cos(yaw)*vy
            down  = -vz
            yaw_rate_ned = -yawspeed
        where yaw is ArduPilot's heading (clockwise from North, radians).
        """
        if not self.connected or not self.vehicle or not self.vehicle.master:
            return

        with self.vehicle._telemetry_lock:
            att = self.vehicle._raw_messages.get("ATTITUDE")
        if att is None:
            return  # no heading yet; do not guess - hold position
        yaw = float(att.yaw)

        c, s = math.cos(yaw), math.sin(yaw)
        v_north = c * vx + s * vy
        v_east = s * vx - c * vy
        v_down = -vz
        yaw_rate = -yawspeed

        # type_mask: ignore position (7), acceleration (448) and yaw (1024); use velocity + yaw rate
        type_mask = 0b010111000111  # 1479
        try:
            self.vehicle.master.mav.set_position_target_local_ned_send(
                0,                                   # time_boot_ms (ignored)
                self.vehicle.target_system,
                self.vehicle.target_component,
                1,                                   # MAV_FRAME_LOCAL_NED
                type_mask,
                0, 0, 0,                             # x, y, z (ignored)
                v_north, v_east, v_down,
                0, 0, 0,                             # accelerations (ignored)
                0,                                   # yaw (ignored)
                yaw_rate,
            )
        except Exception as e:
            print(f'[LISTENER] Error sending velocity setpoint: {e}')

    def _set_mode_guided(self):
        """Switch Pixhawk to GUIDED mode."""
        if not self.connected or not self.vehicle:
            return
        success, msg = self.vehicle.set_mode("GUIDED")
        if success:
            print(f'[LISTENER] Mode set to GUIDED: {msg}')
        else:
            print(f'[LISTENER] Failed to set GUIDED: {msg}')

    def _arm(self):
        """Arm the Pixhawk."""
        if not self.connected or not self.vehicle:
            return
        success, msg = self.vehicle.arm(force=True)
        if success:
            print(f'[LISTENER] Armed: {msg}')
        else:
            print(f'[LISTENER] Arm failed: {msg}')

    def stop(self):
        self._stop_event.set()

    def run(self):
        print(f'[LISTENER] Listening for velocity commands on UDP 0.0.0.0:{self.listen_port}')
        mode_set = False
        armed = False

        try:
            while not self._stop_event.is_set():
                with self.lock:
                    cmd = self.last_cmd
                    cmd_time = self.last_cmd_time

                now = time.time()
                cmd_age = now - cmd_time if cmd_time > 0 else 999.0

                # One-time setup: set GUIDED + arm
                if self.connected and not mode_set:
                    self._set_mode_guided()
                    mode_set = True

                if self.connected and mode_set and not armed:
                    # Wait a moment for mode to take effect, then arm
                    if cmd_age < 2.0:
                        self._arm()
                        armed = True

                # Send velocity if fresh
                if self.connected and armed and cmd and cmd_age < 1.0:
                    action = str(cmd.get('action', '')).lower()
                    if action == 'velocity':
                        vx = float(cmd.get('vx', 0.0))
                        vy = float(cmd.get('vy', 0.0))
                        vz = float(cmd.get('vz', 0.0))
                        yawspeed = float(cmd.get('yawspeed', 0.0))
                        self._send_velocity_setpoint(vx, vy, vz, yawspeed)

                time.sleep(0.05)  # 20 Hz control loop

        except KeyboardInterrupt:
            pass
        finally:
            self._stop_event.set()
            if self._udp_thread and self._udp_thread.is_alive():
                self._udp_thread.join(timeout=1.0)
            self.sock.close()
            if self.vehicle and self._owns_vehicle:
                self.vehicle.disconnect()
            print('[LISTENER] Shutdown complete.')


def main():
    parser = argparse.ArgumentParser(description='MAVLink command listener for Nav2 velocity bridge')
    parser.add_argument('--port', type=str, default='/dev/pixhawk', help='Pixhawk serial port (default: /dev/pixhawk)')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate (default: 115200)')
    parser.add_argument('--listen-port', type=int, default=14553, help='UDP listen port (default: 14553)')
    args = parser.parse_args()

    listener = MavlinkCmdListener(args.port, args.baud, args.listen_port)
    listener.run()


if __name__ == '__main__':
    main()
