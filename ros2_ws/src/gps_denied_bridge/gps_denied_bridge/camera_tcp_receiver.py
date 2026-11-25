#!/usr/bin/env python3

"""
camera_tcp_receiver.py
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
    * Background TCP receiver thread, decoupled from decode/publish:
      the socket is always drained at line rate and only the NEWEST frame is
      decoded and published (stale frames are dropped, never queued)
    * Graceful shutdown
"""

import socket
import struct
import json
import time
import threading
import array
import signal

import cv2
import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

from gps_denied_bridge.metrics_utils import LatencyTracker


def _as_uint8_array(frame):
    """Image.data must get an array.array('B'): assigning bytes/list makes the rclpy
    setter validate every byte in Python (~55 ms per 640x480 frame vs <1 ms)."""
    buf = array.array('B')
    buf.frombytes(np.ascontiguousarray(frame).tobytes())
    return buf


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

        # Depth 1: a slow subscriber gets the latest image, not a 10-deep backlog.
        # Kept RELIABLE so RViz / segmentation_node default subscriptions still match.
        self.image_pub = self.create_publisher(
            Image,
            self.image_topic,
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST, depth=1)
        )
        self.drop_pub = self.create_publisher(Float32, '/metrics/camera/dropped_frames', 10)

        self.declare_parameter('clock_offset_s', 0.0)
        self.lat = LatencyTracker(
            offset_s=float(self.get_parameter('clock_offset_s').value))
        self.lat_pub = self.create_publisher(Float32, '/metrics/camera/latency_ms', 10)
        self.lat_ex_pub = self.create_publisher(Float32, '/metrics/camera/latency_excess_ms', 10)
        self.dec_pub = self.create_publisher(Float32, '/metrics/camera/decode_ms', 10)
        self.bw_pub = self.create_publisher(Float32, '/metrics/camera/bandwidth_mbps', 10)
        self._bytes = 0
        self._bw_t = time.monotonic()

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

        # Latest-frame mailbox between the network thread and the decode thread.
        self._cv = threading.Condition()
        self._pending = None          # (metadata, payload) not yet decoded
        self._dropped = 0             # frames overwritten before they were decoded

        # -----------------------------------------------
        # BACKGROUND THREADS
        # -----------------------------------------------

        self.receiver_thread = threading.Thread(
            target=self.receive_stream,
            daemon=True
        )
        self.decode_thread = threading.Thread(
            target=self.decode_loop,
            daemon=True
        )

        self.receiver_thread.start()
        self.decode_thread.start()

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

        # recv_into a preallocated buffer: no per-chunk bytes objects / copies.
        data = bytearray(length)
        view = memoryview(data)
        got = 0

        while got < length and self.running:

            n = sock.recv_into(view[got:], min(length - got, 262144))

            if not n:
                return None

            got += n

        if got != length:
            return None

        return data

    # ===================================================
    # READ ONE CAMERA FRAME
    # ===================================================

    def read_packet(self, sock):
        """Network only: read one framed packet, return (metadata, payload)."""

        # 8-byte header:
        # [4 bytes metadata length]
        # [4 bytes payload length]

        header = self.recv_all(sock, 8)

        if not header:
            return None

        meta_len, payload_len = struct.unpack(
            '>II', header
        )

        # A desynchronised / foreign stream would otherwise allocate up to 4 GB here.
        if meta_len > 65536 or payload_len > 64 * 1024 * 1024:
            raise ValueError(f'bad frame header (meta {meta_len} B, payload {payload_len} B)')

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

        self._bytes += payload_len + meta_len + 8
        return metadata, payload

    def decode_frame(self, metadata, payload):
        """CPU only: reconstruct the image and update latency metrics."""

        t_dec = time.perf_counter()
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

        decode_ms = (time.perf_counter() - t_dec) * 1e3
        self.dec_pub.publish(Float32(data=float(decode_ms)))

        # Network latency

        t_recv = time.time()

        t_sent = metadata.get('timestamp', t_recv)

        latency = max(
            (t_recv - t_sent) * 1000.0,
            0.0
        )

        if 'timestamp' in metadata:
            self.lat.update(float(metadata['timestamp']))
            self.lat_pub.publish(Float32(data=float(self.lat.raw_ms)))
            self.lat_ex_pub.publish(Float32(data=float(self.lat.excess_ms)))
        dt = time.monotonic() - self._bw_t
        if dt >= 1.0:
            self.bw_pub.publish(Float32(data=float(self._bytes * 8 / dt / 1e6)))
            self._bytes = 0
            self._bw_t = time.monotonic()

        return frame, latency

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

            msg.data = _as_uint8_array(frame)

        elif frame.ndim == 3 and frame.shape[2] == 3:

            msg.height = frame.shape[0]
            msg.width = frame.shape[1]

            msg.encoding = 'bgr8'
            msg.is_bigendian = 0
            msg.step = msg.width * 3

            msg.data = _as_uint8_array(frame)

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

                # The Pi sends 10-30 frames/s; 3 s of silence means a dead link. Drop it
                # so the Pi's reconnect is accepted (listen backlog is only 1).
                client_sock.settimeout(3.0)

                # ---------------------------------------
                # CONTINUOUS FRAME RECEPTION (network only)
                # ---------------------------------------

                while self.running:

                    packet = self.read_packet(client_sock)

                    if packet is None:
                        break

                    with self._cv:
                        if self._pending is not None:
                            self._dropped += 1
                        self._pending = packet
                        self._cv.notify()

            except Exception as e:

                if self.running:

                    self.get_logger().warning(
                        f"Camera stream error: {e}"
                    )

            finally:

                if client_sock is not None:

                    client_sock.close()

                if self.running and client_sock is not None:

                    self.get_logger().warning(
                        "Camera disconnected. Waiting for reconnection..."
                    )

    # ===================================================
    # DECODE + PUBLISH (latest frame only)
    # ===================================================

    def decode_loop(self):

        stream_start = time.monotonic()
        stats_time = stream_start
        self.frame_count = 0

        while self.running:

            with self._cv:
                while self._pending is None and self.running:
                    self._cv.wait(timeout=0.5)
                if not self.running:
                    break
                metadata, payload = self._pending
                self._pending = None

            try:
                frame, latency = self.decode_frame(metadata, payload)
                self.publish_image(frame)
            except Exception as e:
                self.get_logger().warning(f"Camera frame dropped: {e}")
                continue

            self.frame_count += 1
            now = time.monotonic()

            if now - stats_time >= 2.0:
                fps = self.frame_count / (now - stats_time)
                self.drop_pub.publish(Float32(data=float(self._dropped)))
                self.get_logger().info(
                    f"Resolution: {frame.shape[1]}x{frame.shape[0]} | "
                    f"FPS: {fps:.1f} | "
                    f"Latency: {latency:.1f} ms | "
                    f"Dropped (laptop side): {self._dropped}"
                )
                self.frame_count = 0
                stats_time = now

    # ===================================================
    # SHUTDOWN
    # ===================================================

    def destroy_node(self):

        self.get_logger().info(
            "Stopping camera receiver..."
        )

        self.running = False

        with self._cv:
            self._cv.notify_all()

        self.server_sock.close()

        for t in (self.receiver_thread, self.decode_thread):
            if t.is_alive():
                t.join(timeout=2.0)

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

    except (KeyboardInterrupt, ExternalShutdownException):

        pass

    finally:

        signal.signal(signal.SIGINT, signal.SIG_IGN)   # 2nd Ctrl-C: no traceback
        node.destroy_node()

        if rclpy.ok():   # SIGINT already shut the context down
            rclpy.shutdown()


if __name__ == '__main__':

    main()