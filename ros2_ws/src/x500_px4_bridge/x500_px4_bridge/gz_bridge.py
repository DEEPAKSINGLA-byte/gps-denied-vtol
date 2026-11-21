#!/usr/bin/env python3
"""Gazebo Harmonic -> ROS 2 bridge for /clock and the 2D lidar.

Why not ros_gz_bridge? On Humble, the apt package ros-humble-ros-gz-bridge is
built for Gazebo Fortress (ignition-transport11). PX4 runs Gazebo Harmonic
(gz-transport13), and the two transports cannot see each other: the bridge
starts, but nothing ever arrives. This node uses Harmonic's own Python
bindings (python3-gz-transport13, python3-gz-msgs10) instead, so it needs
nothing beyond what PX4's Gazebo install already provides.

Publishes:
  /clock  rosgraph_msgs/Clock     (every Gazebo clock tick)
  /scan   sensor_msgs/LaserScan   (frame_id from the sensor, normally 'link')
"""

import os

# python3-gz-msgs10 ships protobuf code generated for the old system protobuf.
# A newer pip protobuf (e.g. in ~/.local) refuses to load it unless the
# pure-Python implementation is used. Must be set before any *_pb2 import.
os.environ.setdefault('PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION', 'python')

import rclpy  # noqa: E402
from builtin_interfaces.msg import Time  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402

try:
    from gz.msgs10.clock_pb2 import Clock as GzClock  # noqa: E402
    from gz.msgs10.laserscan_pb2 import LaserScan as GzLaserScan  # noqa: E402
    from gz.transport13 import Node as GzNode  # noqa: E402
except ImportError as exc:  # pragma: no cover - depends on the system install
    raise SystemExit(
        f'Gazebo Harmonic Python bindings missing ({exc}). '
        'Install: sudo apt install python3-gz-transport13 python3-gz-msgs10'
    ) from exc


def _stamp(sec: int, nsec: int) -> Time:
    return Time(sec=int(sec), nanosec=int(nsec))


class GzBridge(Node):
    def __init__(self) -> None:
        super().__init__('gz_bridge')
        self.declare_parameter('world', 'baylands')
        self.declare_parameter('model', 'x500_lidar_2d_0')
        self.declare_parameter('gz_scan_topic', '')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('frame_id', '')

        world = self.get_parameter('world').value
        model = self.get_parameter('model').value
        gz_scan = self.get_parameter('gz_scan_topic').value or (
            f'/world/{world}/model/{model}/link/link/sensor/lidar_2d_v2/scan')
        self.frame_id = self.get_parameter('frame_id').value

        # Reliable publishers: work with both reliable and best-effort subscribers.
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.scan_pub = self.create_publisher(
            LaserScan, self.get_parameter('scan_topic').value, 10)

        self.gz = GzNode()
        if not self.gz.subscribe(GzClock, '/clock', self._on_clock):
            self.get_logger().error('Could not subscribe to Gazebo /clock')
        if not self.gz.subscribe(GzLaserScan, gz_scan, self._on_scan):
            self.get_logger().error(f'Could not subscribe to Gazebo {gz_scan}')
        self.scans = 0
        self.get_logger().info(f'Bridging Gazebo /clock and {gz_scan} -> /clock, /scan')

    # gz-transport calls these from its own threads; rclpy publish is thread-safe.
    def _publish(self, pub, msg) -> None:
        try:
            pub.publish(msg)
        except Exception:  # noqa: BLE001 - publisher destroyed while shutting down
            if rclpy.ok():
                raise

    def _on_clock(self, msg: GzClock) -> None:
        self._publish(self.clock_pub, Clock(clock=_stamp(msg.sim.sec, msg.sim.nsec)))

    def _on_scan(self, msg: GzLaserScan) -> None:
        out = LaserScan()
        out.header.stamp = _stamp(msg.header.stamp.sec, msg.header.stamp.nsec)
        out.header.frame_id = self.frame_id or msg.frame
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_step
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = list(msg.ranges)
        out.intensities = list(msg.intensities)
        self._publish(self.scan_pub, out)
        self.scans += 1
        if self.scans == 1:
            self.get_logger().info(
                f'First lidar scan: {len(out.ranges)} beams, frame "{out.header.frame_id}"')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GzBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.gz = None  # stop gz-transport callbacks before the publishers go away
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
