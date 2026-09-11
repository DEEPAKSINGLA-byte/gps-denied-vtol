#!/usr/bin/env python3
"""
x500_px4_bridge.py

ROS 2 Humble bridge:

    /cmd_vel_nav (geometry_msgs/Twist)
             |
             v
    base_footprint-frame velocity
             |
             | current odom->base yaw
             v
    odom-frame ROS velocity
             |
             | ROS -> PX4 axis convention used by tf2.py
             |   PX4 vx = ROS vy
             |   PX4 vy = ROS vx
             |   PX4 vz = -ROS vz
             v
    PX4 TrajectorySetpoint

The node also:
  * maintains PX4 local NED z = -1.5 m;
  * limits horizontal speed to 0.25 m/s initially;
  * stops on stale /cmd_vel_nav commands;
  * applies a simple directional LiDAR emergency stop;
  * publishes OffboardControlMode at 20 Hz;
  * switches to OFFBOARD after a warm-up period and then arms.

IMPORTANT:
This implementation follows the coordinate convention documented in the
current project handoff. Verify the first low-speed flight in Gazebo before
using it on a real vehicle.
"""

import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, TransformException

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand


class X500PX4Bridge(Node):
    def __init__(self) -> None:
        super().__init__('x500_px4_bridge')

        # ---------------- Parameters ----------------
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_nav')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        # Flight parameters
        self.declare_parameter('target_altitude_m', 1.5)
        self.declare_parameter('max_xy_speed_mps', 0.25)
        self.declare_parameter('cmd_timeout_s', 0.50)

        # LiDAR safety parameters
        self.declare_parameter('safety_enabled', True)
        self.declare_parameter('hard_stop_distance_m', 0.45)
        self.declare_parameter('directional_stop_distance_m', 0.65)
        self.declare_parameter('directional_sector_deg', 45.0)

        # PX4 / Offboard parameters
        self.declare_parameter('offboard_rate_hz', 20.0)
        self.declare_parameter('offboard_warmup_setpoints', 20)
        self.declare_parameter('arm_after_setpoints', 30)
        self.declare_parameter('send_commands', True)

        self.target_altitude_m = float(self.get_parameter('target_altitude_m').value)
        self.max_xy_speed = float(self.get_parameter('max_xy_speed_mps').value)
        self.cmd_timeout_s = float(self.get_parameter('cmd_timeout_s').value)

        self.safety_enabled = bool(self.get_parameter('safety_enabled').value)
        self.hard_stop_distance = float(self.get_parameter('hard_stop_distance_m').value)
        self.directional_stop_distance = float(
            self.get_parameter('directional_stop_distance_m').value
        )
        self.directional_sector_rad = math.radians(
            float(self.get_parameter('directional_sector_deg').value)
        )

        self.rate_hz = float(self.get_parameter('offboard_rate_hz').value)
        self.warmup_setpoints = int(self.get_parameter('offboard_warmup_setpoints').value)
        self.arm_after_setpoints = int(self.get_parameter('arm_after_setpoints').value)
        self.send_commands = bool(self.get_parameter('send_commands').value)

        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)

        # ---------------- State ----------------
        self.last_cmd = Twist()
        self.last_cmd_time: Optional[Time] = None
        self.last_scan: Optional[LaserScan] = None
        self.have_scan = False

        self.setpoint_counter = 0
        self.offboard_sent = False
        self.arm_sent = False
        self.last_log_time = self.get_clock().now()

        # ---------------- TF ----------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------------- QoS ----------------
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ---------------- Subscribers ----------------
        self.cmd_sub = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            10,
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            sensor_qos,
        )

        # ---------------- PX4 publishers ----------------
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            px4_qos,
        )

        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            px4_qos,
        )

        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            px4_qos,
        )

        period = 1.0 / max(self.rate_hz, 1.0)
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info('X500 PX4 bridge started')
        self.get_logger().info(
            f'cmd_vel={self.cmd_vel_topic}, scan={self.scan_topic}, '
            f'target_altitude={self.target_altitude_m:.2f} m, '
            f'max_xy_speed={self.max_xy_speed:.2f} m/s'
        )
        self.get_logger().info(
            'PX4 convention: vx_px4=vy_ros_odom, vy_px4=vx_ros_odom, vz_px4=-vz_ros'
        )

    # ------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------
    def cmd_vel_callback(self, msg: Twist) -> None:
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def scan_callback(self, msg: LaserScan) -> None:
        self.last_scan = msg
        self.have_scan = True

    # ------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------
    @staticmethod
    def clamp_xy(vx: float, vy: float, max_speed: float) -> Tuple[float, float]:
        speed = math.hypot(vx, vy)
        if speed <= max_speed or speed < 1e-9:
            return vx, vy
        scale = max_speed / speed
        return vx * scale, vy * scale

    def cmd_is_fresh(self) -> bool:
        if self.last_cmd_time is None:
            return False
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        return age <= self.cmd_timeout_s

    def get_current_yaw(self) -> Optional[float]:
        """Return yaw of base_footprint relative to odom."""
        try:
            # lookup odom <- base transform, i.e. base pose in odom.
            tf_msg = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                Time(),
            )
        except TransformException as exc:
            self._throttled_log(
                f'Waiting for TF {self.odom_frame} -> {self.base_frame}: {exc}'
            )
            return None

        q = tf_msg.transform.rotation
        # Standard quaternion -> yaw extraction.
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def base_to_odom_velocity(
        self, vx_base: float, vy_base: float, yaw: float
    ) -> Tuple[float, float]:
        """Rotate horizontal velocity from base frame into odom frame."""
        c = math.cos(yaw)
        s = math.sin(yaw)
        vx_odom = c * vx_base - s * vy_base
        vy_odom = s * vx_base + c * vy_base
        return vx_odom, vy_odom

    def lidar_emergency_stop(self, vx_base: float, vy_base: float) -> bool:
        """
        Safety check in the LiDAR/base horizontal plane.

        1) Any valid point below hard_stop_distance stops motion.
        2) Otherwise, if an obstacle lies within the direction of commanded
           travel and below directional_stop_distance, stop motion.
        """
        if not self.safety_enabled or not self.have_scan or self.last_scan is None:
            return False

        scan = self.last_scan

        # Hard stop anywhere around the vehicle.
        for r in scan.ranges:
            if math.isfinite(r) and scan.range_min <= r <= self.hard_stop_distance:
                return True

        speed = math.hypot(vx_base, vy_base)
        if speed < 1e-3:
            return False

        motion_angle = math.atan2(vy_base, vx_base)

        angle = scan.angle_min
        for r in scan.ranges:
            if math.isfinite(r) and scan.range_min <= r <= self.directional_stop_distance:
                diff = math.atan2(
                    math.sin(angle - motion_angle),
                    math.cos(angle - motion_angle),
                )
                if abs(diff) <= self.directional_sector_rad:
                    return True
            angle += scan.angle_increment

        return False

    def build_setpoint(self, vx_px4: float, vy_px4: float) -> TrajectorySetpoint:
        msg = TrajectorySetpoint()
        msg.timestamp = self.now_us()

        # PX4 local frame is NED. Keep horizontal position unconstrained and
        # explicitly hold absolute altitude with position z = -1.5 m.
        msg.position = [float('nan'), float('nan'), -abs(self.target_altitude_m)]
        msg.velocity = [float(vx_px4), float(vy_px4), 0.0]

        # No acceleration / jerk constraints.
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.jerk = [float('nan'), float('nan'), float('nan')]

        # We deliberately do not command yaw here. Nav2 is being used for
        # translational exploration; current yaw is read from TF for velocity
        # transformation.
        msg.yaw = float('nan')
        msg.yawspeed = 0.0
        return msg

    def publish_offboard_mode(self) -> None:
        msg = OffboardControlMode()
        msg.timestamp = self.now_us()
        msg.position = True       # for z position hold
        msg.velocity = True       # for x/y velocity control
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_control_mode_pub.publish(msg)

    def publish_vehicle_command(self, command: int, **kwargs: float) -> None:
        msg = VehicleCommand()
        msg.timestamp = self.now_us()
        msg.command = command
        msg.param1 = float(kwargs.get('param1', 0.0))
        msg.param2 = float(kwargs.get('param2', 0.0))
        msg.param3 = float(kwargs.get('param3', 0.0))
        msg.param4 = float(kwargs.get('param4', 0.0))
        msg.param5 = float(kwargs.get('param5', 0.0))
        msg.param6 = float(kwargs.get('param6', 0.0))
        msg.param7 = float(kwargs.get('param7', 0.0))
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)

    def now_us(self) -> int:
        return self.get_clock().now().nanoseconds // 1000

    def _throttled_log(self, text: str) -> None:
        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds * 1e-9 >= 2.0:
            self.get_logger().warn(text)
            self.last_log_time = now

    # ------------------------------------------------------------
    # Main timer
    # ------------------------------------------------------------
    def timer_callback(self) -> None:
        self.publish_offboard_mode()

        # Until we have the current TF yaw, command zero horizontal velocity.
        yaw = self.get_current_yaw()

        vx_base = 0.0
        vy_base = 0.0

        if self.cmd_is_fresh():
            vx_base = float(self.last_cmd.linear.x)
            vy_base = float(self.last_cmd.linear.y)

            vx_base, vy_base = self.clamp_xy(
                vx_base,
                vy_base,
                self.max_xy_speed,
            )

            if self.lidar_emergency_stop(vx_base, vy_base):
                vx_base = 0.0
                vy_base = 0.0
                self._throttled_log('LiDAR safety stop active: horizontal command suppressed')

        if yaw is None:
            vx_px4 = 0.0
            vy_px4 = 0.0
        else:
            vx_odom, vy_odom = self.base_to_odom_velocity(vx_base, vy_base, yaw)

            # ROS/ENU convention used by the current tf2.py:
            #   ROS x = PX4 y
            #   ROS y = PX4 x
            #   ROS z = -PX4 z
            vx_px4 = vy_odom
            vy_px4 = vx_odom

        setpoint = self.build_setpoint(vx_px4, vy_px4)
        self.trajectory_setpoint_pub.publish(setpoint)

        if not self.send_commands:
            return

        self.setpoint_counter += 1

        # First establish the Offboard heartbeat/setpoint stream.
        if (not self.offboard_sent) and self.setpoint_counter >= self.warmup_setpoints:
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                param1=1.0,
                param2=6.0,  # PX4 custom main mode: OFFBOARD
            )
            self.offboard_sent = True
            self.get_logger().info('Requested PX4 OFFBOARD mode')

        # Then arm.
        if (not self.arm_sent) and self.setpoint_counter >= self.arm_after_setpoints:
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                param1=1.0,
            )
            self.arm_sent = True
            self.get_logger().info('Requested PX4 ARM')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = X500PX4Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
