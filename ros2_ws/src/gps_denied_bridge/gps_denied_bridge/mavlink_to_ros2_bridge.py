#!/usr/bin/env python3
"""
mavlink_to_ros2_bridge.py

Receives parsed JSON Pixhawk telemetry over UDP (port 14551) from the Raspberry
Pi companion computer and publishes it as standard ROS 2 topics for Nav2:

  * nav_msgs/Odometry on /odom
    - position.x/y: integrated from optical flow XY velocity (dead-reckoning)
    - position.z:   from barometer / rangefinder altitude
    - orientation:  from attitude roll/pitch/yaw -> quaternion
    - twist.linear: optical flow vx/vy + baro/rangefinder vertical rate
    - twist.angular: from gyro (deg/s -> rad/s)
    - frame_id: odom, child_frame_id: base_link

  * tf2 transform: odom -> base_link (broadcast from odometry pose)

  * geometry_msgs/Vector3Stamped on /drone/optical_flow
    - raw optical flow flow_x/flow_y and ground_distance for logging/debug

This is the GPS-denied state estimation front-end: no GNSS, optical flow +
rangefinder/baro provide local position hold sufficient for Nav2 costmaps.

UDP JSON schema matches the output of PixhawkVehicle.get_telemetry() in
pixhawk_controller.py (telemetry_streamer.py on the Pi).
"""

import json
import math
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Vector3Stamped, TransformStamped
from tf2_ros import TransformBroadcaster


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Convert roll/pitch/yaw (radians) to a ROS Quaternion."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class MavlinkToRos2Bridge(Node):
    def __init__(self):
        super().__init__('mavlink_to_ros2_bridge')

        # ---------------- Parameters ----------------
        self.declare_parameter('listen_port', 14551)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('optical_flow_scale', 1.0)
        self.declare_parameter('z_source', 'rangefinder')  # rangefinder | barometer
        self.declare_parameter('home_alt_m', 0.0)  # baro altitude at takeoff (m, above sea level)

        self.listen_port = int(self.get_parameter('listen_port').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.publish_rate = float(self.get_parameter('publish_rate_hz').value)
        self.of_scale = float(self.get_parameter('optical_flow_scale').value)
        self.z_source = str(self.get_parameter('z_source').value)
        self.home_alt_m = float(self.get_parameter('home_alt_m').value)

        # ---------------- State ----------------
        self.lock = threading.Lock()
        self.latest_telem = None
        self.last_telem_time = None

        # Dead-reckoning position state (integrated from optical flow velocity)
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0
        self.last_integrate_time = None

        # Last known yaw for orienting flow velocity into world frame
        self.last_yaw = 0.0

        # ---------------- Publishers ----------------
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.odom_pub = self.create_publisher(Odometry, '/odom', qos_sensor)
        self.flow_pub = self.create_publisher(
            Vector3Stamped, '/drone/optical_flow', qos_sensor
        )

        # ---------------- TF ----------------
        self.tf_broadcaster = TransformBroadcaster(self)

        # ---------------- UDP socket ----------------
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('0.0.0.0', self.listen_port))
        self.sock.setblocking(False)

        self.get_logger().info(
            f'mavlink_to_ros2_bridge listening on UDP 0.0.0.0:{self.listen_port}'
        )
        self.get_logger().info(
            f'odom_frame={self.odom_frame}, base_frame={self.base_frame}, '
            f'z_source={self.z_source}, of_scale={self.of_scale}'
        )

        # UDP receive thread
        self._stop_event = threading.Event()
        self._udp_thread = threading.Thread(
            target=self._udp_receive_loop, daemon=True, name='MavlinkUDPReceiver'
        )
        self._udp_thread.start()

        # Publish timer
        period = 1.0 / max(self.publish_rate, 1.0)
        self.timer = self.create_timer(period, self._publish_timer_callback)

    # ------------------------------------------------------------
    # UDP receive
    # ------------------------------------------------------------
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
                telem = json.loads(data.decode('utf-8'))
            except Exception:
                continue

            with self.lock:
                self.latest_telem = telem
                self.last_telem_time = time.time()

    # ------------------------------------------------------------
    # Telemetry parsing helpers
    # ------------------------------------------------------------
    @staticmethod
    def _get(nested: dict, *keys, default=None):
        cur = nested
        for k in keys:
            if not isinstance(cur, dict) or k not in cur:
                return default
            cur = cur[k]
        return cur

    def _integrate_position(self, telem: dict, now: float):
        """Integrate optical flow velocity into world-frame X/Y position."""
        att = telem.get('attitude') or {}
        flow = telem.get('optical_flow') or {}
        lpos = telem.get('local_position_ned') or {}

        # Prefer EKF local position if available (already fused on Pixhawk)
        if lpos and all(k in lpos for k in ('x_m', 'y_m', 'z_m')):
            with self.lock:
                self.pos_x = float(lpos['x_m'])
                self.pos_y = float(lpos['y_m'])
                self.pos_z = float(lpos['z_m'])
                self.last_integrate_time = now
                yaw_deg = att.get('yaw_deg', 0.0)
                self.last_yaw = math.radians(yaw_deg)
            return

        # Otherwise dead-reckon from optical flow velocity
        flow_x = flow.get('flow_x', 0.0)
        flow_y = flow.get('flow_y', 0.0)
        if flow_x is None:
            flow_x = 0.0
        if flow_y is None:
            flow_y = 0.0

        # Apply scale and rotate from body frame to world frame using yaw
        vx_body = float(flow_x) * self.of_scale
        vy_body = float(flow_y) * self.of_scale

        yaw = self.last_yaw
        c = math.cos(yaw)
        s = math.sin(yaw)
        vx_world = c * vx_body - s * vy_body
        vy_world = s * vx_body + c * vy_body

        with self.lock:
            if self.last_integrate_time is not None:
                dt = now - self.last_integrate_time
                if 0.0 < dt < 1.0:  # ignore large gaps
                    self.pos_x += vx_world * dt
                    self.pos_y += vy_world * dt
            self.last_integrate_time = now

            yaw_deg = att.get('yaw_deg', 0.0)
            self.last_yaw = math.radians(yaw_deg)

    def _get_altitude(self, telem: dict) -> float:
        """Get Z altitude (positive up) from rangefinder or barometer."""
        if self.z_source == 'rangefinder':
            rf = telem.get('rangefinder') or {}
            d = rf.get('distance_m', None)
            if d is not None and d > 0:
                # Rangefinder measures distance to ground; height above ground = +d
                return float(d)
        # Fallback to barometer / VFR_HUD altitude
        baro = telem.get('barometer') or {}
        if baro and baro.get('baro_alt_m') is not None:
            # baro_alt_m is absolute pressure altitude above sea level; subtract
            # takeoff elevation to get height above home (positive up).
            return float(baro['baro_alt_m']) - self.home_alt_m
        hud = telem.get('vfr_hud') or {}
        if hud and hud.get('alt_m') is not None:
            # VFR_HUD alt is height above home (positive up).
            return float(hud['alt_m'])
        return self.pos_z  # hold last

    # ------------------------------------------------------------
    # Publish timer
    # ------------------------------------------------------------
    def _publish_timer_callback(self):
        with self.lock:
            telem = self.latest_telem
            if telem is None:
                return
            now = time.time()

        self._integrate_position(telem, now)

        with self.lock:
            pos_x = self.pos_x
            pos_y = self.pos_y
            pos_z = self._get_altitude(telem)
            # Override z with integrated baro/rangefinder value
            self.pos_z = pos_z

            att = telem.get('attitude') or {}
            imu = telem.get('imu') or {}
            flow = telem.get('optical_flow') or {}
            lpos = telem.get('local_position_ned') or {}

            roll = math.radians(att.get('roll_deg', 0.0) or 0.0)
            pitch = math.radians(att.get('pitch_deg', 0.0) or 0.0)
            yaw = math.radians(att.get('yaw_deg', 0.0) or 0.0)

            q = quaternion_from_euler(roll, pitch, yaw)

            # Twist: linear from optical flow + vertical rate, angular from gyro
            vx_body = float(flow.get('flow_x', 0.0) or 0.0) * self.of_scale
            vy_body = float(flow.get('flow_y', 0.0) or 0.0) * self.of_scale

            # Vertical velocity: prefer EKF local position vz, else baro rate (approx 0)
            if lpos and lpos.get('vz_ms') is not None:
                vz = float(lpos['vz_ms'])  # NED vz (down positive); negate for up
            else:
                vz = 0.0

            gx = float(imu.get('gyro_x_dps', 0.0) or 0.0)
            gy = float(imu.get('gyro_y_dps', 0.0) or 0.0)
            gz = float(imu.get('gyro_z_dps', 0.0) or 0.0)
            deg2rad = math.pi / 180.0

        # Build odometry message
        odom = Odometry()
        stamp = self.get_clock().now().to_msg()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = pos_x
        odom.pose.pose.position.y = pos_y
        odom.pose.pose.position.z = pos_z
        odom.pose.pose.orientation = q

        # Covariance (tune as needed)
        odom.pose.covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.10, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.05,
        ]

        odom.twist.twist.linear.x = vx_body
        odom.twist.twist.linear.y = vy_body
        odom.twist.twist.linear.z = -vz  # NED -> up positive
        odom.twist.twist.angular.x = gx * deg2rad
        odom.twist.twist.angular.y = gy * deg2rad
        odom.twist.twist.angular.z = gz * deg2rad

        odom.twist.covariance = [
            0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.02, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.02, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.05,
        ]

        self.odom_pub.publish(odom)

        # Broadcast TF: odom -> base_link
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = pos_x
        t.transform.translation.y = pos_y
        t.transform.translation.z = pos_z
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

        # Publish optical flow for debugging
        flow_msg = Vector3Stamped()
        flow_msg.header.stamp = stamp
        flow_msg.header.frame_id = self.base_frame
        flow_msg.vector.x = vx_body
        flow_msg.vector.y = vy_body
        rf = telem.get('rangefinder') or {}
        flow_msg.vector.z = float(rf.get('distance_m', 0.0) or 0.0)
        self.flow_pub.publish(flow_msg)

    def destroy_node(self):
        self._stop_event.set()
        if self._udp_thread and self._udp_thread.is_alive():
            self._udp_thread.join(timeout=1.0)
        self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MavlinkToRos2Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
