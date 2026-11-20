#!/usr/bin/env python3
"""
lidar_udp_receiver.py - Ground ROS 2 Receiver for YDLIDAR UDP Stream

Listens on UDP port 14552 and publishes standard sensor_msgs/LaserScan to /scan.
Standalone copy of the gps_denied_bridge lidar_udp_receiver node (keep in sync).

Receives JSON scan packets from raspberry_pi/lidar_udp_streamer.py and converts
them to LaserScan messages for Nav2 costmap consumption.
"""

import math
import socket
import json
import struct
import threading

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

try:
    from gps_denied_bridge.metrics_utils import LatencyTracker
except ImportError:  # standalone use without the gps_denied_bridge package sourced
    import sys, os
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..',
                                    'ros2_ws', 'src', 'gps_denied_bridge'))
    from gps_denied_bridge.metrics_utils import LatencyTracker

LIDAR_MAGIC = b'YDL1'
_HDR = struct.Struct('<4sdH')


def decode_scan(data):
    """Return (timestamp|None, ranges_m ndarray, intensities ndarray) for either the
    compact binary format (lidar_udp_streamer.py default) or the legacy JSON one."""
    if data[:4] == LIDAR_MAGIC:
        _, ts, n = _HDR.unpack_from(data)
        o = _HDR.size
        ranges = np.frombuffer(data, '<u2', n, o).astype(np.float32) / 1000.0
        inten = np.frombuffer(data, np.uint8, n, o + 2 * n).astype(np.float32)
        return ts, ranges, inten
    payload = json.loads(data.decode())
    ranges = np.asarray(payload.get('ranges', [0.0] * 360), np.float32)
    inten = np.asarray(payload.get('intensities', [0] * len(ranges)), np.float32)
    if len(inten) != len(ranges):
        inten = np.zeros_like(ranges)
    return payload.get('timestamp'), ranges, inten


class LidarUDPReceiverNode(Node):
    def __init__(self):
        super().__init__('lidar_udp_receiver_node')

        self.declare_parameter('listen_port', 14552)
        self.declare_parameter('frame_id', 'laser_frame')
        self.declare_parameter('scan_topic', 'scan')
        # YDLIDAR reports angle clockwise from its zero mark; ROS wants CCW. Raw index k
        # (deg, CW) -> ROS angle -(k + offset). Rotate with this or the base_link->laser TF yaw.
        self.declare_parameter('clockwise_input', True)
        self.declare_parameter('angle_offset_deg', 0.0)

        self.listen_port = int(self.get_parameter('listen_port').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.cw = bool(self.get_parameter('clockwise_input').value)
        self.ang_off = int(round(float(self.get_parameter('angle_offset_deg').value)))

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.scan_pub = self.create_publisher(LaserScan, self.scan_topic, qos_sensor)

        # Pi->laptop one-way latency (needs chrony/NTP on both for absolute
        # values; 'excess' is clock-offset free).
        self.declare_parameter('clock_offset_s', 0.0)
        self.lat = LatencyTracker(
            offset_s=float(self.get_parameter('clock_offset_s').value))
        self.lat_pub = self.create_publisher(Float32, '/metrics/lidar/latency_ms', 10)
        self.lat_ex_pub = self.create_publisher(Float32, '/metrics/lidar/latency_excess_ms', 10)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('0.0.0.0', self.listen_port))
        self.sock.settimeout(0.5)

        # Precomputed raw-bin -> ROS-order index map (see _to_ros_order).
        i = np.arange(360)
        self._src = ((180 - i - self.ang_off) if self.cw else (i - 180 - self.ang_off)) % 360

        self.get_logger().info(
            f'Listening for LiDAR UDP packets on port {self.listen_port} -> /{self.scan_topic}'
        )
        # Blocking receive thread: publishes the instant a scan arrives instead of
        # waiting for the next 10 ms timer tick of a polling loop.
        self.running = True
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()

    def _to_ros_order(self, ranges, intensities):
        """Reorder 1-degree raw bins (index = sensor angle) to ROS order: index i is
        angle -pi + i deg, counter-clockwise positive, 0 rad = +x of the laser frame."""
        if len(ranges) != 360:
            return ranges, intensities
        return ranges[self._src], intensities[self._src]

    def _latest_datagram(self):
        """Block for one datagram, then drain any backlog so only the newest scan is
        used; stale scans queued during a Wi-Fi hiccup are dropped, not replayed."""
        data, _ = self.sock.recvfrom(65535)
        self.sock.setblocking(False)
        try:
            while True:
                data, _ = self.sock.recvfrom(65535)
        except (BlockingIOError, InterruptedError):
            pass
        finally:
            self.sock.settimeout(0.5)
        return data

    def _rx_loop(self):
        while self.running and rclpy.ok():
            try:
                data = self._latest_datagram()
            except socket.timeout:
                continue
            except OSError as e:
                if self.running:
                    self.get_logger().warn(f'UDP recv error: {e}')
                continue
            try:
                self._handle(data)
            except Exception as e:
                self.get_logger().warn(f'Error parsing UDP scan packet: {e}')

    def _handle(self, data):
        ts, ranges, intensities = decode_scan(data)
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = self.frame_id
        scan_msg.angle_min = -math.pi
        scan_msg.angle_increment = (2.0 * math.pi) / 360.0
        # 360 bins from -pi: last bin is at pi - increment (angle_max is inclusive)
        scan_msg.angle_max = math.pi - scan_msg.angle_increment
        scan_msg.time_increment = 0.1 / 360.0
        scan_msg.scan_time = 0.1
        scan_msg.range_min = 0.12
        scan_msg.range_max = 16.0

        ranges, intensities = self._to_ros_order(ranges, intensities)
        ranges = np.where(ranges > 0.0, ranges, np.inf).astype(np.float32)
        scan_msg.ranges = ranges.tolist()
        scan_msg.intensities = intensities.tolist()

        self.scan_pub.publish(scan_msg)

        if ts is not None:
            self.lat.update(float(ts))
            self.lat_pub.publish(Float32(data=float(self.lat.raw_ms)))
            self.lat_ex_pub.publish(Float32(data=float(self.lat.excess_ms)))

    def destroy_node(self):
        self.running = False
        self.rx_thread.join(timeout=1.0)
        self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LidarUDPReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
