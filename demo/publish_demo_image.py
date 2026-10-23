#!/usr/bin/env python3
"""Publish a still image repeatedly as a ROS 2 camera stream for demos."""

import argparse

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class DemoImagePublisher(Node):
    """Publish an image on a sensor_msgs/Image topic at a modest rate."""

    def __init__(self, image_path, topic, rate_hz):
        super().__init__('demo_image_publisher')
        self.publisher = self.create_publisher(Image, topic, 1)
        self.bridge = CvBridge()
        self.frame = cv2.imread(image_path)
        if self.frame is None:
            raise ValueError(f'Unable to load image: {image_path}')
        self.timer = self.create_timer(1.0 / rate_hz, self.publish_image)
        self.get_logger().info(f'Publishing {image_path} on {topic}')

    def publish_image(self):
        """Publish one frame with the current ROS timestamp."""
        msg = self.bridge.cv2_to_imgmsg(self.frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'demo_camera'
        self.publisher.publish(msg)


def main():
    """Parse arguments and publish the prepared image until interrupted."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('image', help='Path to a demo image')
    parser.add_argument('--topic', default='/camera/image_raw')
    parser.add_argument('--rate', type=float, default=1.0)
    args = parser.parse_args()
    if args.rate <= 0:
        parser.error('--rate must be greater than zero')

    rclpy.init()
    node = DemoImagePublisher(args.image, args.topic, args.rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
