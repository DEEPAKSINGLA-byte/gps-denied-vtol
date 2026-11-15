#!/usr/bin/env python3
"""
Ground ROS 2 Receiver for YDLIDAR UDP Stream
Listens on UDP port 14552 and publishes standard sensor_msgs/LaserScan to /scan.
"""
import math
import socket
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

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

        self.listen_port = self.get_parameter('listen_port').value
        self.frame_id = self.get_parameter('frame_id').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.cw = bool(self.get_parameter('clockwise_input').value)
        self.ang_off = int(round(float(self.get_parameter('angle_offset_deg').value)))

        self.scan_pub = self.create_publisher(LaserScan, self.scan_topic, 10)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', self.listen_port))
        self.sock.setblocking(False)

        self.get_logger().info(f"Listening for LiDAR UDP packets on port {self.listen_port} -> /{self.scan_topic}")
        self.timer = self.create_timer(0.01, self.poll_udp)

    def _to_ros_order(self, ranges, intensities):
        """Reorder 1-degree raw bins (index = sensor angle) to ROS order: index i is
        angle -pi + i deg, counter-clockwise positive, 0 rad = +x of the laser frame."""
        n = len(ranges)
        if n != 360:
            return ranges, intensities
        if self.cw:
            src = [(180 - i - self.ang_off) % 360 for i in range(360)]
        else:
            src = [(i - 180 - self.ang_off) % 360 for i in range(360)]
        inten = intensities if len(intensities) == 360 else [0] * 360
        return [ranges[k] for k in src], [inten[k] for k in src]

    def poll_udp(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(65535)
                payload = json.loads(data.decode())
                
                scan_msg = LaserScan()
                scan_msg.header.stamp = self.get_clock().now().to_msg()
                scan_msg.header.frame_id = self.frame_id
                scan_msg.angle_min = -math.pi
                scan_msg.angle_max = math.pi
                scan_msg.angle_increment = (2 * math.pi) / 360.0
                scan_msg.time_increment = 0.1 / 360.0
                scan_msg.scan_time = 0.1
                scan_msg.range_min = 0.12
                scan_msg.range_max = 16.0

                ranges = payload.get("ranges", [0.0] * 360)
                intensities = payload.get("intensities", [0] * 360)

                ranges, intensities = self._to_ros_order(ranges, intensities)
                scan_msg.ranges = [r if r > 0.0 else float('inf') for r in ranges]
                scan_msg.intensities = [float(i) for i in intensities]

                self.scan_pub.publish(scan_msg)

            except BlockingIOError:
                break
            except Exception as e:
                self.get_logger().warn(f"Error parsing UDP scan packet: {e}")
                break

    def destroy_node(self):
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
