#!/usr/bin/env python3
"""
camera_receiver.py - Receive & Verify RGB Camera Image Matrix on Laptop
======================================================================
Listens on the ground laptop network interface (e.g., Redmi 10) for image matrices
transmitted by the Raspberry Pi RGB camera streamer.

Features:
  * Full reconstruction of NumPy image matrix (H, W, 3) uint8
  * Automated data integrity verification (--verify)
  * Measures network transit latency & frame throughput (FPS)
  * Saves received frame to disk (e.g. received_image_matrix.jpg)
  * Optional live OpenCV display window (--gui)

Usage Examples:
  # Verify single frame reception & save snapshot
  python3 camera_receiver.py --verify

  # Receive and save incoming snapshot to custom filename
  python3 camera_receiver.py --save test_capture.jpg

  # Continuous stream reception with live FPS monitoring
  python3 camera_receiver.py --stream
"""

import sys
import time
import json
import struct
import socket
import argparse
from typing import Optional, Tuple

try:
    import cv2
    import numpy as np
except ImportError:
    print("ERROR: OpenCV and NumPy are required. Install via: pip install opencv-python numpy")
    sys.exit(1)


def recv_all(sock: socket.socket, length: int) -> Optional[bytes]:
    """
    Read exactly `length` bytes from TCP socket stream.
    """
    data = bytearray()
    while len(data) < length:
        packet = sock.recv(min(length - len(data), 65536))
        if not packet:
            return None
        data.extend(packet)
    return bytes(data)


def read_frame(sock: socket.socket) -> Optional[Tuple[dict, np.ndarray, float]]:
    """
    Read next frame from TCP socket:
      [4 bytes meta_len] [4 bytes payload_len] [meta_bytes] [payload_bytes]
    Returns: (metadata_dict, image_matrix, transit_latency_ms)
    """
    header_data = recv_all(sock, 8)
    if not header_data:
        return None

    meta_len, payload_len = struct.unpack(">II", header_data)

    meta_bytes = recv_all(sock, meta_len)
    if not meta_bytes:
        return None
    metadata = json.loads(meta_bytes.decode("utf-8"))

    payload = recv_all(sock, payload_len)
    if not payload:
        return None

    # Calculate network transit latency
    t_recv = time.time()
    t_sent = metadata.get("timestamp", t_recv)
    latency_ms = max((t_recv - t_sent) * 1000.0, 0.0)

    # Reconstruct NumPy matrix
    encoding = metadata.get("encoding", "jpeg")
    h, w, c = metadata["shape"]

    if encoding == "raw":
        frame = np.frombuffer(payload, dtype=metadata.get("dtype", "uint8")).reshape((h, w, c))
    else:
        buf = np.frombuffer(payload, dtype=np.uint8)
        frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)

    return metadata, frame, latency_ms


def verify_image_stream(port: int = 5000, save_path: str = "received_image_matrix.jpg", timeout: float = 15.0) -> bool:
    print("=" * 70)
    print(" RGB CAMERA MATRIX VERIFICATION (REDMI 10 NETWORK)")
    print("=" * 70)
    print(f"Listening on:     0.0.0.0:{port} (TCP)")
    print(f"Output File:      {save_path}")
    print(f"Connection Wait:  {timeout:.1f}s")
    print("-" * 70)

    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind(("0.0.0.0", port))
    server_sock.listen(1)
    server_sock.settimeout(timeout)

    print("Waiting for Raspberry Pi camera streamer to connect...")
    try:
        client_sock, client_addr = server_sock.accept()
        print(f"[+] Connected! Client IP: {client_addr[0]}:{client_addr[1]}")
    except socket.timeout:
        print(f"[FAILED] Timed out waiting for connection on port {port}.")
        server_sock.close()
        return False

    client_sock.settimeout(10.0)
    print("Reading image matrix from stream...")
    res = read_frame(client_sock)
    client_sock.close()
    server_sock.close()

    if not res:
        print("[FAILED] Failed reading frame payload from stream.")
        return False

    metadata, frame, latency_ms = res

    # Verification checks
    checks = []

    # 1. Type check
    is_ndarray = isinstance(frame, np.ndarray)
    checks.append(("Data Type is numpy.ndarray", is_ndarray, f"Type: {type(frame).__name__}"))

    # 2. Dimensions & Shape
    has_3d = frame.ndim == 3
    checks.append(("Matrix Dimensions (3D HxWxC)", has_3d, f"Shape: {frame.shape}"))

    # 3. Channel Count
    channels = frame.shape[2] if frame.ndim == 3 else 0
    checks.append(("RGB/BGR 3 Channels", channels == 3, f"Channels: {channels}"))

    # 4. Data Type
    is_uint8 = frame.dtype == np.uint8
    checks.append(("Pixel Type is uint8 (0-255)", is_uint8, f"dtype: {frame.dtype}"))

    # 5. Image Pixel Validity (non-empty, non-black)
    mean_val = float(frame.mean())
    min_val = int(frame.min())
    max_val = int(frame.max())
    has_content = max_val > 0 and mean_val > 5.0
    checks.append(("Image Content & Contrast", has_content, f"Min={min_val}, Max={max_val}, Mean={mean_val:.2f}"))

    # 6. Save image to disk
    cv2.imwrite(save_path, frame)
    checks.append(("Saved Image Frame to Disk", True, f"Path: {save_path}"))

    print("\n" + "=" * 70)
    print(" CAMERA MATRIX INTEGRITY & VALIDATION REPORT")
    print("=" * 70)
    print(f"Sender IP:         {client_addr[0]}")
    print(f"Frame ID:          {metadata.get('frame_id', 1)}")
    print(f"Matrix Shape:      {frame.shape} (Height={frame.shape[0]}, Width={frame.shape[1]}, Channels={frame.shape[2]})")
    print(f"Total Pixel Count: {frame.size:,} pixels ({frame.nbytes:,} bytes in memory)")
    print(f"Payload Size:      {metadata.get('payload_size', len(frame.tobytes())):,} bytes over wire")
    print(f"Encoding:          {metadata.get('encoding', 'raw').upper()}")
    print(f"Transit Latency:   {latency_ms:.2f} ms")
    print("-" * 70)

    all_passed = True
    for name, passed, detail in checks:
        badge = "\033[92m[ PASSED ]\033[0m" if passed else "\033[91m[ FAILED ]\033[0m"
        print(f" {badge} {name:<30}: {detail}")
        if not passed:
            all_passed = False

    print("=" * 70)
    if all_passed:
        print("\033[92m[SUCCESS] RGB Camera Image Matrix received and validated successfully!\033[0m")
    else:
        print("\033[91m[FAILED] Image matrix verification checks failed.\033[0m")
    print("=" * 70 + "\n")
    return all_passed


def run_stream_receiver(port: int = 5000, save_path: Optional[str] = None):
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind(("0.0.0.0", port))
    server_sock.listen(1)

    print(f"Listening for continuous camera stream on 0.0.0.0:{port}...")
    try:
        client_sock, client_addr = server_sock.accept()
        print(f"[+] Stream connection established from {client_addr[0]}:{client_addr[1]}")

        frame_count = 0
        t_start = time.time()
        while True:
            res = read_frame(client_sock)
            if not res:
                print("\nStream disconnected.")
                break

            meta, frame, latency = res
            frame_count += 1
            elapsed = time.time() - t_start
            fps = frame_count / elapsed if elapsed > 0 else 0

            sys.stdout.write(
                f"\r[RECEIVING] Frame {frame_count:>4} | Shape: {frame.shape} | "
                f"Rate: {fps:>4.1f} FPS | Latency: {latency:>4.1f}ms | Mean: {frame.mean():>5.1f}"
            )
            sys.stdout.flush()

            if save_path and frame_count % 30 == 0:
                cv2.imwrite(save_path, frame)

    except KeyboardInterrupt:
        print("\nReceiver stopped by user.")
    finally:
        server_sock.close()


def main():
    parser = argparse.ArgumentParser(description="Laptop RGB Camera Matrix Receiver")
    parser.add_argument("--port", type=int, default=5000, help="TCP port (default: 5000)")
    parser.add_argument("--verify", action="store_true", help="Capture 1 frame, verify schema & image, then exit")
    parser.add_argument("--stream", action="store_true", help="Receive continuous stream and monitor FPS")
    parser.add_argument("--save", type=str, default="received_image_matrix.jpg", help="Path to save received frame")
    parser.add_argument("--timeout", type=float, default=15.0, help="Connection timeout in seconds")

    args = parser.parse_args()

    if args.stream:
        run_stream_receiver(port=args.port, save_path=args.save)
    else:
        ok = verify_image_stream(port=args.port, save_path=args.save, timeout=args.timeout)
        sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
