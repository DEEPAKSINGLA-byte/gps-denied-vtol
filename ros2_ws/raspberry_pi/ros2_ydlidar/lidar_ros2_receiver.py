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

        self.listen_port = self.get_parameter('listen_port').value
        self.frame_id = self.get_parameter('frame_id').value
        self.scan_topic = self.get_parameter('scan_topic').value

        self.scan_pub = self.create_publisher(LaserScan, self.scan_topic, 10)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', self.listen_port))
        self.sock.setblocking(False)

        self.get_logger().info(f"Listening for LiDAR UDP packets on port {self.listen_port} -> /{self.scan_topic}")
        self.timer = self.create_wall_timer(0.01, self.poll_udp)

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
