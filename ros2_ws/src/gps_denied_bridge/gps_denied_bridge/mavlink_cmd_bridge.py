#!/usr/bin/env python3
"""
mavlink_cmd_bridge.py

Subscribes to Nav2 /cmd_vel (geometry_msgs/Twist) and forwards commands to the
Pixhawk flight controller via the Raspberry Pi companion computer over UDP.

Two command channels:

  1. Discrete commands (arm/disarm/set_mode/emergency_kill) -> UDP port 14552
     Handled by the existing Pi telemetry_streamer.py _command_listener_worker.
     JSON format: {"action": "arm", "force": true}

  2. Continuous velocity setpoints -> UDP port 14553
     Handled by a new Pi-side mavlink_cmd_listener.py that uses PixhawkVehicle
     to send MAV_CMD_DO_SET_MODE (GUIDED) + MAV_CMD_NAV_VELOCITY setpoints.
     JSON format: {"action": "velocity", "vx": 0.5, "vy": 0.0, "vz": 0.0,
                   "yawspeed": 0.0, "frame": "body"}

Coordinate convention:
  /cmd_vel linear.x = forward (+X in base frame), linear.y = left (+Y),
  linear.z = up (+Z), angular.z = yaw rate.
  These are sent as-is to the Pi listener which converts to PX4/ArduPilot
  NED/body-frame setpoints.
"""

import json
import math
import os
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist


class MavlinkCmdBridge(Node):
    def __init__(self):
        super().__init__('mavlink_cmd_bridge')

        # ---------------- Parameters ----------------
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('pi_ip', os.environ.get('GPS_DENIED_PI_IP', '10.184.3.36'))
        self.declare_parameter('discrete_port', 14552)          # Pi cmd listener
        self.declare_parameter('velocity_port', 14553)          # new Pi velocity listener
        self.declare_parameter('cmd_timeout_s', 0.50)           # stale command timeout
        self.declare_parameter('max_xy_speed_mps', 0.5)         # safety speed clamp
        self.declare_parameter('auto_arm', False)               # auto-arm on first cmd
        self.declare_parameter('auto_guided', True)             # auto-switch to GUIDED

        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.pi_ip = str(self.get_parameter('pi_ip').value)
        self.discrete_port = int(self.get_parameter('discrete_port').value)
        self.velocity_port = int(self.get_parameter('velocity_port').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout_s').value)
        self.max_xy_speed = float(self.get_parameter('max_xy_speed_mps').value)
        self.auto_arm = bool(self.get_parameter('auto_arm').value)
        self.auto_guided = bool(self.get_parameter('auto_guided').value)

        # ---------------- State ----------------
        self.lock = threading.Lock()
        self.last_cmd = Twist()
        self.last_cmd_time = None
        self.mode_sent = False
        self.arm_sent = False
        self.velocity_streaming = False

        # ---------------- QoS ----------------
        qos_cmd = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ---------------- Subscribers ----------------
        self.cmd_sub = self.create_subscription(
            Twist, self.cmd_vel_topic, self.cmd_vel_callback, qos_cmd
        )

        # ---------------- UDP sockets ----------------
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        self.get_logger().info(
            f'mavlink_cmd_bridge: cmd_vel={self.cmd_vel_topic} -> '
            f'Pi {self.pi_ip}:{self.discrete_port} (discrete), '
            f'{self.velocity_port} (velocity)'
        )
        self.get_logger().info(
            f'auto_guided={self.auto_guided}, auto_arm={self.auto_arm}, '
            f'max_xy_speed={self.max_xy_speed} m/s'
        )

        # Publish timer: send velocity setpoints at 20 Hz
        self.timer = self.create_timer(0.05, self._timer_callback)

    # ------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------
    def cmd_vel_callback(self, msg: Twist):
        with self.lock:
            self.last_cmd = msg
            self.last_cmd_time = self.get_clock().now()

    def _cmd_is_fresh(self) -> bool:
        if self.last_cmd_time is None:
            return False
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        return age <= self.cmd_timeout

    def _clamp_xy(self, vx: float, vy: float):
        speed = math.hypot(vx, vy)
        if speed > self.max_xy_speed and speed > 1e-9:
            scale = self.max_xy_speed / speed
            return vx * scale, vy * scale
        return vx, vy

    # ------------------------------------------------------------
    # Timer: send discrete setup + continuous velocity
    # ------------------------------------------------------------
    def _timer_callback(self):
        with self.lock:
            cmd = self.last_cmd
            fresh = self._cmd_is_fresh()
            mode_sent = self.mode_sent
            arm_sent = self.arm_sent
            vx = float(cmd.linear.x)
            vy = float(cmd.linear.y)
            vz = float(cmd.linear.z)
            yaw_rate = float(cmd.angular.z)

        if not fresh:
            # Stale command: stop
            vx = 0.0
            vy = 0.0
            vz = 0.0
            yaw_rate = 0.0

        # Clamp horizontal speed for safety
        vx, vy = self._clamp_xy(vx, vy)

        # One-time: switch to GUIDED mode
        if self.auto_guided and not mode_sent:
            self._send_discrete({'action': 'set_mode', 'mode': 'GUIDED'})
            self.mode_sent = True
            self.get_logger().info('Sent SET_MODE GUIDED to Pixhawk')

        # One-time: arm
        if self.auto_arm and not arm_sent and self.mode_sent:
            self._send_discrete({'action': 'arm', 'force': True})
            self.arm_sent = True
            self.get_logger().info('Sent ARM to Pixhawk')

        # Continuous velocity setpoint
        if abs(vx) > 1e-3 or abs(vy) > 1e-3 or abs(vz) > 1e-3 or abs(yaw_rate) > 1e-3:
            if not self.velocity_streaming:
                self.get_logger().info('Streaming velocity setpoints to Pixhawk')
                self.velocity_streaming = True
            self._send_velocity(vx, vy, vz, yaw_rate)
        elif self.velocity_streaming:
            # Send zero to hold position
            self._send_velocity(0.0, 0.0, 0.0, 0.0)

    # ------------------------------------------------------------
    # UDP send helpers
    # ------------------------------------------------------------
    def _send_discrete(self, cmd_dict: dict):
        try:
            payload = json.dumps(cmd_dict).encode('utf-8')
            self.sock.sendto(payload, (self.pi_ip, self.discrete_port))
        except Exception as e:
            self.get_logger().warn(f'Failed to send discrete command: {e}')

    def _send_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float):
        cmd_dict = {
            'action': 'velocity',
            'vx': round(vx, 3),
            'vy': round(vy, 3),
            'vz': round(vz, 3),
            'yawspeed': round(yaw_rate, 3),
            'frame': 'body',
            'timestamp': time.time(),
        }
        try:
            payload = json.dumps(cmd_dict).encode('utf-8')
            self.sock.sendto(payload, (self.pi_ip, self.velocity_port))
        except Exception as e:
            self.get_logger().warn(f'Failed to send velocity command: {e}')

    def destroy_node(self):
        self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MavlinkCmdBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
