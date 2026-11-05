#!/usr/bin/env python3
"""
camera_streamer.py - Stream RGB Camera Image Matrix from Raspberry Pi to Laptop
================================================================================
Captures live frames from the RGB camera connected to the Raspberry Pi (/dev/video0)
and transmits the image matrix over the network (e.g., Redmi 10) to the laptop.

Supports:
  * Raw Image Matrix: Full uncompressed NumPy array (H, W, 3) uint8 bytes.
  * JPEG Encoded Stream: Compressed matrix payload for smooth, low-latency 30 FPS video.
  * One-shot snapshot or continuous streaming.

Usage Examples:
  # Send single image matrix snapshot to laptop (Redmi 10)
  python3 camera_streamer.py --target-ip 10.184.3.231 --snapshot

  # Stream live camera frames continuously
  python3 camera_streamer.py --target-ip 10.184.3.231 --stream

  # Stream uncompressed raw matrix
  python3 camera_streamer.py --target-ip 10.184.3.231 --mode raw --snapshot
"""

import sys
import os
import time
import json
import struct
import socket
import argparse
from typing import Optional

try:
    import cv2
    import numpy as np
except ImportError:
    print("ERROR: OpenCV and NumPy are required. Install via: pip install opencv-python numpy")
    sys.exit(1)


def get_default_target_ip() -> str:
    ssh_client = os.environ.get("SSH_CLIENT")
    if ssh_client:
        return ssh_client.split()[0]
    return "10.184.3.231"


def init_camera(device_index: int = 0, width: int = 640, height: int = 480):
    print(f"Opening camera device /dev/video{device_index}...")
    cap = cv2.VideoCapture(device_index, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"[!] CAP_V4L2 failed, trying default backend for /dev/video{device_index}...")
        cap = cv2.VideoCapture(device_index)

    if not cap.isOpened():
        return None

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    # Warmup camera sensor (discard first 5 frames to allow auto-exposure to stabilize)
    print("Warming up camera sensor...")
    for _ in range(5):
        cap.read()
        time.sleep(0.05)

    return cap


def send_frame(sock: socket.socket, frame: np.ndarray, frame_id: int, mode: str = "jpeg", jpeg_quality: int = 85) -> bool:
    """
    Serialize and transmit an image matrix with metadata header over TCP socket.
    """
    timestamp = time.time()
    h, w, c = frame.shape

    if mode == "raw":
        payload = frame.tobytes()
        encoding = "raw"
    else:
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality]
        success, encimg = cv2.imencode(".jpg", frame, encode_param)
        if not success:
            return False
        payload = encimg.tobytes()
        encoding = "jpeg"

    metadata = {
        "frame_id": frame_id,
        "timestamp": timestamp,
        "shape": [h, w, c],
        "dtype": str(frame.dtype),
        "encoding": encoding,
        "payload_size": len(payload),
        "mean_intensity": round(float(frame.mean()), 2),
    }

    meta_bytes = json.dumps(metadata).encode("utf-8")
    header = struct.pack(">II", len(meta_bytes), len(payload))

    try:
        sock.sendall(header + meta_bytes + payload)
        return True
    except Exception as e:
        print(f"[!] Socket transmission error: {e}")
        return False


def run_streamer(
    target_ip: str,
    port: int = 5000,
    device: int = 0,
    mode: str = "jpeg",
    snapshot: bool = False,
    fps_limit: float = 20.0
):
    print("=" * 68)
    print(" RASPBERRY PI RGB CAMERA MATRIX STREAMER")
    print("=" * 68)
    print(f"Target Laptop:    {target_ip}:{port}")
    print(f"Camera Device:    /dev/video{device}")
    print(f"Encoding Mode:    {mode.upper()}")
    print(f"Transmission:     {'SINGLE SNAPSHOT' if snapshot else 'CONTINUOUS STREAM'}")
    print("=" * 68)

    cap = init_camera(device_index=device)
    if not cap:
        print(f"[FAILED] Could not open camera device /dev/video{device}", file=sys.stderr)
        sys.exit(1)

    ret, test_frame = cap.read()
    if not ret or test_frame is None:
        print("[FAILED] Could not read test frame from camera.", file=sys.stderr)
        cap.release()
        sys.exit(1)

    print(f"[+] Camera active! Frame shape: {test_frame.shape}, dtype: {test_frame.dtype}, mean: {test_frame.mean():.2f}")

    print(f"\nConnecting to laptop at {target_ip}:{port}...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect((target_ip, port))
        print(f"[+] TCP connection established with {target_ip}:{port}!")
    except Exception as e:
        print(f"[FAILED] Could not connect to {target_ip}:{port}: {e}")
        print("  Ensure camera_receiver.py is running on your laptop first!")
        cap.release()
        sock.close()
        sys.exit(1)

    interval = 1.0 / max(fps_limit, 1.0)
    frame_id = 0
    t_start = time.time()

    try:
        if snapshot:
            ret, frame = cap.read()
            if ret and frame is not None:
                ok = send_frame(sock, frame, frame_id=1, mode=mode)
                if ok:
                    print(f"[SUCCESS] Sent image matrix snapshot: shape={frame.shape}, size={len(frame.tobytes())} bytes")
                else:
                    print("[FAILED] Failed sending snapshot.")
        else:
            print("Streaming live frames... Press Ctrl+C to stop.")
            while True:
                loop_start = time.time()
                ret, frame = cap.read()
                if not ret or frame is None:
                    continue

                frame_id += 1
                ok = send_frame(sock, frame, frame_id=frame_id, mode=mode)
                if not ok:
                    break

                elapsed = time.time() - t_start
                rate = frame_id / elapsed if elapsed > 0 else 0
                sys.stdout.write(f"\r[STREAMING] Frame {frame_id:>4} | Shape: {frame.shape} | Rate: {rate:>4.1f} FPS | Mean: {frame.mean():>5.1f}")
                sys.stdout.flush()

                delay = interval - (time.time() - loop_start)
                if delay > 0:
                    time.sleep(delay)

    except KeyboardInterrupt:
        print("\nStopping camera streamer...")
    finally:
        cap.release()
        sock.close()
        print("Camera streamer closed.")


def main():
    default_ip = get_default_target_ip()
    parser = argparse.ArgumentParser(description="Stream RGB Camera Matrix to Laptop")
    parser.add_argument("--target-ip", type=str, default=default_ip, help=f"Target laptop IP (default: {default_ip})")
    parser.add_argument("--port", type=int, default=5000, help="TCP port (default: 5000)")
    parser.add_argument("--device", type=int, default=0, help="V4L2 camera device index (default: 0)")
    parser.add_argument("--mode", type=str, choices=["jpeg", "raw"], default="jpeg", help="Matrix encoding mode (default: jpeg)")
    parser.add_argument("--snapshot", action="store_true", help="Send single verified frame snapshot and exit")
    parser.add_argument("--stream", action="store_true", help="Stream frames continuously")
    parser.add_argument("--fps", type=float, default=20.0, help="Maximum stream FPS limit (default: 20.0)")

    args = parser.parse_args()

    is_snapshot = not args.stream
    run_streamer(
        target_ip=args.target_ip,
        port=args.port,
        device=args.device,
        mode=args.mode,
        snapshot=is_snapshot,
        fps_limit=args.fps
    )


if __name__ == "__main__":
    main()
