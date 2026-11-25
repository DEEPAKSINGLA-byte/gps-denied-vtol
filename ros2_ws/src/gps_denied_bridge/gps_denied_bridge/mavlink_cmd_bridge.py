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

Latency: each /cmd_vel is forwarded the moment it arrives (no wait for a timer tick).
The 20 Hz timer only keeps the setpoint stream alive / sends the stale-command stop when
Nav2 goes quiet. Packets are DSCP CS6 marked (Wi-Fi WMM voice queue).
"""

import json
import math
import os
import signal
import socket
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
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
        # GUIDED/ARM are requested when Nav2 STARTS commanding motion (not at launch, when the
        # Pi may still be offline and the packet would be lost), once per command session.
        # A new session begins after Nav2 has been quiet this long, so a pilot who switches
        # out of GUIDED mid-mission is not overridden while the same goal is still running.
        self.declare_parameter('session_gap_s', 3.0)

        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.pi_ip = str(self.get_parameter('pi_ip').value)
        self.discrete_port = int(self.get_parameter('discrete_port').value)
        self.velocity_port = int(self.get_parameter('velocity_port').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout_s').value)
        self.max_xy_speed = float(self.get_parameter('max_xy_speed_mps').value)
        self.auto_arm = bool(self.get_parameter('auto_arm').value)
        self.auto_guided = bool(self.get_parameter('auto_guided').value)
        self.session_gap_s = float(self.get_parameter('session_gap_s').value)

        # ---------------- State ----------------
        self.lock = threading.Lock()
        self.last_cmd = Twist()
        self.last_cmd_time = None
        self.mode_sent = False
        self.arm_sent = False
        self.velocity_streaming = False
        self._last_motion = None   # monotonic time of the last fresh non-zero command

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
        try:  # DSCP CS6 -> WMM voice: commands jump ahead of queued camera traffic
            self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_TOS, 0xC0)
        except OSError:
            pass

        self.get_logger().info(
            f'mavlink_cmd_bridge: cmd_vel={self.cmd_vel_topic} -> '
            f'Pi {self.pi_ip}:{self.discrete_port} (discrete), '
            f'{self.velocity_port} (velocity)'
        )
        self.get_logger().info(
            f'auto_guided={self.auto_guided}, auto_arm={self.auto_arm}, '
            f'max_xy_speed={self.max_xy_speed} m/s'
        )

        # Keepalive timer (20 Hz): commands are sent on arrival in cmd_vel_callback; the
        # timer only fires a send when nothing went out for a full period.
        self._period = 0.05
        self._last_tick = 0.0
        self.timer = self.create_timer(self._period, self._timer_callback)

    # ------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------
    def cmd_vel_callback(self, msg: Twist):
        with self.lock:
            self.last_cmd = msg
            self.last_cmd_time = self.get_clock().now()
        # Forward immediately. Safe: the default single-threaded executor never runs this
        # concurrently with _timer_callback.
        self._tick()

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
        # 10% slack so a timer tick right after a cmd-triggered send is skipped
        if time.monotonic() - self._last_tick >= 0.9 * self._period:
            self._tick()

    def _tick(self):
        self._last_tick = time.monotonic()
        with self.lock:
            cmd = self.last_cmd
            fresh = self._cmd_is_fresh()
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
        moving = abs(vx) > 1e-3 or abs(vy) > 1e-3 or abs(vz) > 1e-3 or abs(yaw_rate) > 1e-3

        now = time.monotonic()
        if moving:
            self._last_motion = now
            # Start of a command session: GUIDED first, then (opt-in) ARM. The Pi listener
            # handles them in order and only forwards velocity while armed in GUIDED.
            if self.auto_guided and not self.mode_sent:
                self._send_discrete({'action': 'set_mode', 'mode': 'GUIDED'})
                self.mode_sent = True
                self.get_logger().info('Nav2 started commanding: sent SET_MODE GUIDED to Pixhawk')
            if self.auto_arm and not self.arm_sent:
                self._send_discrete({'action': 'arm', 'force': True})
                self.arm_sent = True
                self.get_logger().info('Sent ARM to Pixhawk')
        elif (self.mode_sent or self.arm_sent) and self._last_motion is not None \
                and now - self._last_motion > self.session_gap_s:
            self.mode_sent = self.arm_sent = False   # next motion = new session

        # Continuous velocity setpoint
        if moving:
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
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)   # 2nd Ctrl-C: no traceback
        node.destroy_node()
        if rclpy.ok():   # SIGINT already shut the context down
            rclpy.shutdown()


if __name__ == '__main__':
    main()
