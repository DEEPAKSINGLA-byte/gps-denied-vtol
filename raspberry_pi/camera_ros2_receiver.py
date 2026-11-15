#!/usr/bin/env python3

"""
camera_ros2_receiver.py
=========================================================
Ground Station RGB Camera ROS 2 Receiver

Receives continuous TCP image stream from Raspberry Pi
and publishes sensor_msgs/Image for RViz2.

Pipeline:
    Raspberry Pi -> TCP -> Ground Laptop -> ROS 2 -> RViz2

Features:
    * Continuous TCP image reception
    * JPEG and RAW image reconstruction
    * ROS 2 Image publishing
    * Live FPS and latency monitoring
    * Background TCP receiver thread
    * Graceful shutdown
"""

import socket
import struct
import json
import time
import threading

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header


class CameraROS2Receiver(Node):

    def __init__(self):

        super().__init__('camera_ros2_receiver')

        # -----------------------------------------------
        # PARAMETERS
        # -----------------------------------------------

        self.declare_parameter('port', 5000)
        self.declare_parameter('frame_id', 'camera_frame')
        self.declare_parameter('image_topic', '/camera/image_raw')

        self.port = self.get_parameter('port').value
        self.frame_id = self.get_parameter('frame_id').value
        self.image_topic = self.get_parameter('image_topic').value

        # -----------------------------------------------
        # ROS PUBLISHER
        # -----------------------------------------------

        self.image_pub = self.create_publisher(
            Image,
            self.image_topic,
            10
        )

        # -----------------------------------------------
        # TCP SERVER
        # -----------------------------------------------

        self.server_sock = socket.socket(
            socket.AF_INET,
            socket.SOCK_STREAM
        )

        self.server_sock.setsockopt(
            socket.SOL_SOCKET,
            socket.SO_REUSEADDR,
            1
        )

        self.server_sock.bind(('0.0.0.0', self.port))
        self.server_sock.listen(1)

        self.server_sock.settimeout(1.0)

        # -----------------------------------------------
        # STATISTICS
        # -----------------------------------------------

        self.frame_count = 0
        self.start_time = time.monotonic()

        self.running = True

        # -----------------------------------------------
        # BACKGROUND THREAD
        # -----------------------------------------------

        self.receiver_thread = threading.Thread(
            target=self.receive_stream,
            daemon=True
        )

        self.receiver_thread.start()

        self.get_logger().info("=" * 55)
        self.get_logger().info(" RGB CAMERA ROS 2 RECEIVER")
        self.get_logger().info("=" * 55)

        self.get_logger().info(
            f"Listening on TCP port: {self.port}"
        )

        self.get_logger().info(
            f"ROS Topic: {self.image_topic}"
        )

        self.get_logger().info(
            f"Frame ID: {self.frame_id}"
        )

        self.get_logger().info(
            "Waiting for Raspberry Pi camera streamer..."
        )

    # ===================================================
    # RECEIVE EXACT NUMBER OF BYTES
    # ===================================================

    def recv_all(self, sock, length):

        data = bytearray()

        while len(data) < length and self.running:

            packet = sock.recv(
                min(length - len(data), 65536)
            )

            if not packet:
                return None

            data.extend(packet)

        if len(data) != length:
            return None

        return bytes(data)

    # ===================================================
    # READ ONE CAMERA FRAME
    # ===================================================

    def read_frame(self, sock):

        # 8-byte header:
        # [4 bytes metadata length]
        # [4 bytes payload length]

        header = self.recv_all(sock, 8)

        if not header:
            return None

        meta_len, payload_len = struct.unpack(
            '>II', header
        )

        # Read metadata

        meta_bytes = self.recv_all(sock, meta_len)

        if not meta_bytes:
            return None

        metadata = json.loads(
            meta_bytes.decode('utf-8')
        )

        # Read image payload

        payload = self.recv_all(sock, payload_len)

        if not payload:
            return None

        # -----------------------------------------------
        # RECONSTRUCT IMAGE
        # -----------------------------------------------

        encoding = metadata.get('encoding', 'jpeg')

        h, w, c = metadata['shape']

        if encoding == 'raw':

            frame = np.frombuffer(
                payload,
                dtype=metadata.get('dtype', 'uint8')
            ).reshape((h, w, c))

        else:

            buffer = np.frombuffer(
                payload,
                dtype=np.uint8
            )

            frame = cv2.imdecode(
                buffer,
                cv2.IMREAD_COLOR
            )

        if frame is None:
            raise ValueError("Failed to decode camera frame")

        # Network latency

        t_recv = time.time()

        t_sent = metadata.get('timestamp', t_recv)

        latency = max(
            (t_recv - t_sent) * 1000.0,
            0.0
        )

        return metadata, frame, latency

    # ===================================================
    # CONVERT OPENCV IMAGE TO ROS IMAGE
    # ===================================================

    def publish_image(self, frame):

        msg = Image()

        msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        msg.header.frame_id = self.frame_id

        # OpenCV images are generally BGR

        if frame.ndim == 2:

            msg.height = frame.shape[0]
            msg.width = frame.shape[1]

            msg.encoding = 'mono8'
            msg.is_bigendian = 0
            msg.step = msg.width

            msg.data = frame.tobytes()

        elif frame.ndim == 3 and frame.shape[2] == 3:

            msg.height = frame.shape[0]
            msg.width = frame.shape[1]

            msg.encoding = 'bgr8'
            msg.is_bigendian = 0
            msg.step = msg.width * 3

            msg.data = np.ascontiguousarray(
                frame
            ).tobytes()

        else:

            raise ValueError(
                f"Unsupported image shape: {frame.shape}"
            )

        self.image_pub.publish(msg)

    # ===================================================
    # CONTINUOUS STREAM RECEIVER
    # ===================================================

    def receive_stream(self):

        while self.running:

            client_sock = None

            try:

                # Wait for Raspberry Pi connection

                try:

                    client_sock, address = (
                        self.server_sock.accept()
                    )

                except socket.timeout:

                    continue

                self.get_logger().info(
                    f"Camera connected: {address[0]}"
                )

                client_sock.settimeout(10.0)

                self.frame_count = 0

                stream_start = time.monotonic()
                stats_time = stream_start

                # ---------------------------------------
                # CONTINUOUS FRAME RECEPTION
                # ---------------------------------------

                while self.running:

                    result = self.read_frame(client_sock)

                    if result is None:
                        break

                    metadata, frame, latency = result

                    # Publish ROS image

                    self.publish_image(frame)

                    self.frame_count += 1

                    now = time.monotonic()

                    elapsed = now - stream_start

                    fps = (
                        self.frame_count / elapsed
                        if elapsed > 0 else 0
                    )

                    # -----------------------------------
                    # PERIODIC STATISTICS
                    # -----------------------------------

                    if now - stats_time >= 2.0:

                        self.get_logger().info(
                            f"Frame: {self.frame_count} | "
                            f"Resolution: {frame.shape[1]}x"
                            f"{frame.shape[0]} | "
                            f"FPS: {fps:.1f} | "
                            f"Latency: {latency:.1f} ms"
                        )

                        stats_time = now

            except Exception as e:

                if self.running:

                    self.get_logger().warning(
                        f"Camera stream error: {e}"
                    )

            finally:

                if client_sock is not None:

                    client_sock.close()

                if self.running:

                    self.get_logger().warning(
                        "Camera disconnected. Waiting for reconnection..."
                    )

    # ===================================================
    # SHUTDOWN
    # ===================================================

    def destroy_node(self):

        self.get_logger().info(
            "Stopping camera receiver..."
        )

        self.running = False

        self.server_sock.close()

        if self.receiver_thread.is_alive():

            self.receiver_thread.join(timeout=2.0)

        self.get_logger().info(
            "Camera receiver stopped."
        )

        super().destroy_node()


# =======================================================
# MAIN
# =======================================================

def main(args=None):

    rclpy.init(args=args)

    node = CameraROS2Receiver()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()