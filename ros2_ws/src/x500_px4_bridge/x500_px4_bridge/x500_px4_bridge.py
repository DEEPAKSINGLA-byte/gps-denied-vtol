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
  * removes motion towards obstacles closer than ~0.65 m (lidar safety);
  * publishes OffboardControlMode at 20 Hz;
  * switches to OFFBOARD after a warm-up period and then arms, retrying
    once a second until PX4's vehicle_status confirms both (PX4 may
    still be booting when this node starts).

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

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus


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
        self.declare_parameter('safety_enabled', False)
        self.declare_parameter('hard_stop_distance_m', 0.45)
        self.declare_parameter('directional_stop_distance_m', 0.65)
        self.declare_parameter('directional_sector_deg', 45.0)  # unused, kept for old configs
        self.declare_parameter('repulsion_speed_mps', 0.15)
        # Turn to face the direction of travel (the 2D lidar covers 270 degrees
        # and is blind behind), slowing down while the turn is large.
        self.declare_parameter('yaw_follows_motion', True)
        self.declare_parameter('yaw_follow_min_speed_mps', 0.12)

        # PX4 / Offboard parameters
        self.declare_parameter('offboard_rate_hz', 20.0)
        self.declare_parameter('offboard_warmup_setpoints', 20)
        self.declare_parameter('arm_after_setpoints', 30)
        self.declare_parameter('send_commands', True)
        self.declare_parameter('command_retry_s', 1.0)
        self.declare_parameter('command_retry_timeout_s', 120.0)
        # Simulation convenience: if PX4 drops out of OFFBOARD or disarms (e.g. a
        # failsafe after a short /clock gap), ask for OFFBOARD + ARM again.
        # Set false on anything that is not a simulator.
        self.declare_parameter('keep_offboard', True)

        self.target_altitude_m = float(self.get_parameter('target_altitude_m').value)
        self.max_xy_speed = float(self.get_parameter('max_xy_speed_mps').value)
        self.cmd_timeout_s = float(self.get_parameter('cmd_timeout_s').value)

        self.safety_enabled = bool(self.get_parameter('safety_enabled').value)
        self.hard_stop_distance = float(self.get_parameter('hard_stop_distance_m').value)
        self.directional_stop_distance = float(
            self.get_parameter('directional_stop_distance_m').value
        )
        self.repulsion_speed = float(self.get_parameter('repulsion_speed_mps').value)
        self.yaw_follows_motion = bool(self.get_parameter('yaw_follows_motion').value)
        self.yaw_follow_min_speed = float(self.get_parameter('yaw_follow_min_speed_mps').value)
        self.target_yaw_ned = float('nan')
        self.directional_sector_rad = math.radians(
            float(self.get_parameter('directional_sector_deg').value)
        )

        self.rate_hz = float(self.get_parameter('offboard_rate_hz').value)
        self.warmup_setpoints = int(self.get_parameter('offboard_warmup_setpoints').value)
        self.arm_after_setpoints = int(self.get_parameter('arm_after_setpoints').value)
        self.send_commands = bool(self.get_parameter('send_commands').value)
        self.command_retry_s = float(self.get_parameter('command_retry_s').value)
        self.command_retry_timeout_s = float(
            self.get_parameter('command_retry_timeout_s').value
        )
        self.keep_offboard = bool(self.get_parameter('keep_offboard').value)

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
        self.vehicle_status: Optional[VehicleStatus] = None
        self.takeoff_done = False
        self.in_control = False
        self.gave_up = False
        self.first_command_time: Optional[Time] = None
        self.last_command_time: Optional[Time] = None
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

        # PX4 >= 1.16 versions this topic (vehicle_status_v4 for px4_msgs
        # MESSAGE_VERSION 4); older firmware uses the plain name. Listen to both.
        self.status_subs = [
            self.create_subscription(VehicleStatus, topic, self.status_callback, px4_qos)
            for topic in ('/fmu/out/vehicle_status_v4', '/fmu/out/vehicle_status')
        ]

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
            f'max_xy_speed={self.max_xy_speed:.2f} m/s, '
            f'safety_enabled={self.safety_enabled}'
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

    def status_callback(self, msg: VehicleStatus) -> None:
        self.vehicle_status = msg

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

    def lidar_safety_filter(self, vx_base: float, vy_base: float) -> Tuple[float, float, bool]:
        """Remove the part of the command that points at a close obstacle.

        For every lidar return closer than directional_stop_distance_m (or
        hard_stop_distance_m, whichever is larger), the velocity component
        towards that point is removed (vector projection); motion along or
        away from it is kept. So the drone can still slide along a wall or
        back out of a doorway. (An earlier version stopped all motion as soon
        as anything was within reach, which left the drone frozen next to a
        door frame until it drifted into it.)

        Anything closer than hard_stop_distance_m also adds a gentle push
        away from it (up to repulsion_speed_mps), even while hovering with no
        command, so the drone never drifts into what it is next to.

        Returns (vx, vy, limited).
        """
        if not self.safety_enabled or not self.have_scan or self.last_scan is None:
            return vx_base, vy_base, False

        scan = self.last_scan
        reach = max(self.hard_stop_distance, self.directional_stop_distance)
        close = []
        angle = scan.angle_min
        for r in scan.ranges:
            if math.isfinite(r) and scan.range_min <= r <= reach:
                close.append((r, math.cos(angle), math.sin(angle)))
            angle += scan.angle_increment
        if not close:
            return vx_base, vy_base, False

        limited = False
        for _ in range(2):  # a second pass catches components re-added by the first
            for _, ux, uy in close:
                toward = vx_base * ux + vy_base * uy
                if toward > 1e-6:
                    vx_base -= toward * ux
                    vy_base -= toward * uy
                    limited = True

        px = py = 0.0
        for r, ux, uy in close:
            if r < self.hard_stop_distance:
                k = 1.0 - r / self.hard_stop_distance
                px -= k * ux
                py -= k * uy
        push = math.hypot(px, py)
        if push > 1e-6:
            scale = min(push, 1.0) * self.repulsion_speed / push
            vx_base += px * scale
            vy_base += py * scale
            limited = True
        return vx_base, vy_base, limited

    def face_direction_of_travel(
        self, vx_px4: float, vy_px4: float, yaw_enu: float
    ) -> Tuple[float, float]:
        """Set target_yaw_ned to the travel direction; slow down while turning.

        PX4 heading is NED: 0 = north, +90 deg = east, i.e. atan2(v_east, v_north).
        While the heading error is large the lidar does not see the way ahead
        yet, so speed is scaled by cos(error) (at least 25 %), and to 10 %
        when the target is more than 90 deg away (turn first, then go).
        """
        speed = math.hypot(vx_px4, vy_px4)
        if speed < self.yaw_follow_min_speed:
            return vx_px4, vy_px4  # keep the last heading while (nearly) hovering
        self.target_yaw_ned = math.atan2(vy_px4, vx_px4)
        yaw_ned = math.pi / 2.0 - yaw_enu
        err = math.atan2(math.sin(self.target_yaw_ned - yaw_ned),
                         math.cos(self.target_yaw_ned - yaw_ned))
        scale = max(0.25, math.cos(err)) if abs(err) <= math.pi / 2 else 0.1
        return vx_px4 * scale, vy_px4 * scale

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

        # Heading: with yaw_follows_motion, the direction of travel (so the
        # lidar, which is blind over the rear 90 degrees, looks where the
        # drone is going); otherwise NaN = PX4 keeps the current heading.
        msg.yaw = float(self.target_yaw_ned)
        msg.yawspeed = float('nan')
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

        # Also while hovering without a command: it may push away from a wall.
        if self.takeoff_done:
            vx_base, vy_base, limited = self.lidar_safety_filter(vx_base, vy_base)
            if limited:
                self._throttled_log('LiDAR safety: motion towards a close obstacle removed')

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

            if self.yaw_follows_motion:
                vx_px4, vy_px4 = self.face_direction_of_travel(vx_px4, vy_px4, yaw)

        setpoint = self.build_setpoint(vx_px4, vy_px4)
        self.trajectory_setpoint_pub.publish(setpoint)

        if not self.send_commands:
            return

        self.setpoint_counter += 1
        if self.gave_up or self.setpoint_counter < self.warmup_setpoints:
            return

        status = self.vehicle_status
        offboard = (status is not None
                    and status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        armed = (status is not None
                 and status.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        if offboard and armed:
            if not self.in_control:
                self.get_logger().info(
                    'PX4 is in OFFBOARD and armed: taking off' if not self.takeoff_done
                    else 'Back in OFFBOARD and armed')
            self.in_control = True
            self.takeoff_done = True
            self.first_command_time = None
            return

        if self.in_control:
            self.in_control = False
            if not self.keep_offboard:
                self.get_logger().warn('PX4 left OFFBOARD/armed; not taking control back '
                                       '(keep_offboard is false)')
                self.gave_up = True
                return
            self.get_logger().warn(
                'PX4 left OFFBOARD/armed (failsafe, e.g. setpoints stopped for a moment); '
                'requesting it again')
        if self.takeoff_done and not self.keep_offboard:
            return

        now = self.get_clock().now()
        if self.first_command_time is None:
            self.first_command_time = now
        elapsed = (now - self.first_command_time).nanoseconds * 1e-9
        if elapsed > self.command_retry_timeout_s:
            self.get_logger().error(
                'PX4 did not reach OFFBOARD + armed within '
                f'{self.command_retry_timeout_s:.0f} s; giving up (check the pxh> output)')
            self.gave_up = True  # stop retrying
            return
        if (self.last_command_time is not None
                and (now - self.last_command_time).nanoseconds * 1e-9 < self.command_retry_s):
            return
        self.last_command_time = now

        if not offboard:
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                param1=1.0,
                param2=6.0,  # PX4 custom main mode: OFFBOARD
            )
            if not self.offboard_sent:
                self.get_logger().info('Requested PX4 OFFBOARD mode')
            self.offboard_sent = True

        # Arm once the warm-up count is reached (PX4 accepts arm then OFFBOARD
        # too, so this does not have to wait for the mode switch).
        if not armed and self.setpoint_counter >= self.arm_after_setpoints:
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                param1=1.0,
            )
            if not self.arm_sent:
                self.get_logger().info('Requested PX4 ARM')
            self.arm_sent = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = X500PX4Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
