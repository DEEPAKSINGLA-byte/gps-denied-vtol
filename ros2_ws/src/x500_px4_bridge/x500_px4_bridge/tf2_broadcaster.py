#!/usr/bin/env python3
"""PX4 odometry -> ROS TF and /odom.

TF published (REP 105 layout):

    odom -> base_footprint   x, y and heading only (z = 0, level): the
                             drone's shadow on the floor. 2D SLAM and Nav2
                             use this frame, so map/odom/base_footprint all
                             stay at ground level.
    base_footprint -> base_link
                             height, roll and pitch: the real body pose.
                             Sensors (the lidar) hang off base_link.

Why the split: with height and tilt in base_footprint, slam_toolbox puts the
map frame at whatever height the drone had when mapping started, so obstacle
heights in the global costmap meant nothing and floor hits from a tilted
lidar were marked as obstacles under the drone.

/odom (nav_msgs/Odometry): full 3D pose of base_link in odom (height
included, handy for plots), twist in the body frame.
"""
import math

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from rclpy.node import Node


def _quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


class PX4OdomToBaseLinkBroadcaster(Node):
    def __init__(self):
        super().__init__('px4_odom_tf_broadcaster')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('footprint_frame', 'base_footprint')
        self.declare_parameter('base_frame', 'base_link')
        self.odom_frame = self.get_parameter('odom_frame').value
        self.footprint_frame = self.get_parameter('footprint_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.subscription = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_callback, qos_profile)

    def odom_callback(self, msg):
        if np.isnan(msg.position[0]) or np.isnan(msg.q[0]):
            return
        stamp = self.get_clock().now().to_msg()

        # PX4 NED / FRD -> ROS ENU / FLU.
        x, y, z = float(msg.position[1]), float(msg.position[0]), -float(msg.position[2])
        q_w, q_x, q_y, q_z = (float(v) for v in msg.q)
        full = (0.70710678 * (q_w + q_z), 0.70710678 * (q_x + q_y),
                0.70710678 * (q_x - q_y), 0.70710678 * (q_w - q_z))  # (w, x, y, z)
        w, qx, qy, qz = full
        yaw = math.atan2(2.0 * (w * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        q_yaw = (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))
        # Remaining rotation (roll, pitch) relative to the level footprint.
        q_tilt = _quat_mul((q_yaw[0], 0.0, 0.0, -q_yaw[3]), full)

        foot = TransformStamped()
        foot.header.stamp = stamp
        foot.header.frame_id = self.odom_frame
        foot.child_frame_id = self.footprint_frame
        foot.transform.translation.x = x
        foot.transform.translation.y = y
        foot.transform.rotation.w, foot.transform.rotation.z = q_yaw[0], q_yaw[3]

        body = TransformStamped()
        body.header.stamp = stamp
        body.header.frame_id = self.footprint_frame
        body.child_frame_id = self.base_frame
        body.transform.translation.z = z
        (body.transform.rotation.w, body.transform.rotation.x,
         body.transform.rotation.y, body.transform.rotation.z) = q_tilt

        self.tf_broadcaster.sendTransform([foot, body])

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
        odom.pose.pose.position.z = z
        (odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
         odom.pose.pose.orientation.y, odom.pose.pose.orientation.z) = full
        # PX4 velocity is NED in the world frame; ROS twist is in the body frame.
        if not np.isnan(msg.velocity[0]):
            vx_enu, vy_enu = float(msg.velocity[1]), float(msg.velocity[0])
            odom.twist.twist.linear.x = math.cos(yaw) * vx_enu + math.sin(yaw) * vy_enu
            odom.twist.twist.linear.y = -math.sin(yaw) * vx_enu + math.cos(yaw) * vy_enu
            odom.twist.twist.linear.z = -float(msg.velocity[2])
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = PX4OdomToBaseLinkBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
