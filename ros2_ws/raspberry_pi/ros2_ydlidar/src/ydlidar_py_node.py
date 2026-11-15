#!/usr/bin/env python3
"""
Standalone Python ROS 2 Node for YDLIDAR G2 / G2B
Publishes LaserScan messages to /scan topic directly using serial communication.
"""
import math
import struct
import time
import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class YDlidarPyNode(Node):
    def __init__(self):
        super().__init__('ydlidar_py_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'laser_frame')
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('range_min', 0.12)
        self.declare_parameter('range_max', 16.0)

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)

        self.scan_pub = self.create_publisher(LaserScan, self.scan_topic, 10)
        self.get_logger().info(f"Connecting to YDLIDAR G2 on {self.port} at {self.baudrate} baud...")

        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1.0)
            self.ser.dtr = False
            self.ser.rts = True
            time.sleep(0.3)
            # Start scan command (0xA5, 0x60)
            self.ser.write(bytes([0xA5, 0x60]))
            header = self.ser.read(7)
            self.get_logger().info(f"LiDAR handshake header: {header.hex()}")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port {self.port}: {e}")
            raise e

        self.current_scan_ranges = [float('inf')] * 360
        self.current_scan_intensities = [0.0] * 360
        self.last_scan_time = self.get_clock().now()

        # Run parser loop in timer
        self.timer = self.create_wall_timer(0.001, self.read_and_parse)

    def read_and_parse(self):
        try:
            # Look for sync bytes 0xAA 0x55
            b1 = self.ser.read(1)
            if not b1 or b1[0] != 0xAA:
                return
            b2 = self.ser.read(1)
            if not b2 or b2[0] != 0x55:
                return

            # Read packet header
            header_data = self.ser.read(8)
            if len(header_data) < 8:
                return

            ct, lsn, fsa, lsa, cs = struct.unpack('<BBHHH', header_data)
            point_data = self.ser.read(lsn * 3)  # 3 bytes per point for G2 with intensity
            if len(point_data) < lsn * 3:
                return

            # Parse start and end angles (unit: 1/64 degree)
            start_angle = (fsa >> 1) / 64.0
            end_angle = (lsa >> 1) / 64.0
            angle_diff = (end_angle - start_angle) % 360.0

            is_first_packet = (ct & 0x01) == 0x01

            if is_first_packet:
                # Publish completed full revolution
                now = self.get_clock().now()
                scan_duration = (now - self.last_scan_time).nanoseconds / 1e9
                self.last_scan_time = now

                msg = LaserScan()
                msg.header.stamp = now.to_msg()
                msg.header.frame_id = self.frame_id
                msg.angle_min = -math.pi
                msg.angle_max = math.pi
                msg.angle_increment = (2 * math.pi) / 360.0
                msg.scan_time = scan_duration if scan_duration > 0 else 0.1
                msg.time_increment = msg.scan_time / 360.0
                msg.range_min = self.range_min
                msg.range_max = self.range_max
                msg.ranges = list(self.current_scan_ranges)
                msg.intensities = list(self.current_scan_intensities)

                self.scan_pub.publish(msg)
                self.current_scan_ranges = [float('inf')] * 360
                self.current_scan_intensities = [0.0] * 360

            # Parse points in this packet
            for i in range(lsn):
                sample = point_data[i * 3: (i + 1) * 3]
                intensity = sample[0]
                dist_raw = sample[1] | (sample[2] << 8)
                dist_m = (dist_raw / 4.0) / 1000.0  # mm to m

                if lsn > 1:
                    angle = start_angle + (angle_diff / (lsn - 1)) * i
                else:
                    angle = start_angle
                angle = angle % 360.0

                idx = int(angle) % 360
                if self.range_min <= dist_m <= self.range_max:
                    self.current_scan_ranges[idx] = dist_m
                    self.current_scan_intensities[idx] = float(intensity)

        except Exception as e:
            pass

    def destroy_node(self):
        try:
            # Stop scan command (0xA5, 0x65)
            self.ser.write(bytes([0xA5, 0x65]))
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = YDlidarPyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
