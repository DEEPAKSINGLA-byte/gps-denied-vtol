#!/usr/bin/env python3
"""
camera_streamer.py - Ultra-Low-Latency RGB Camera Streamer from Raspberry Pi to Laptop
======================================================================================
Captures live frames from the RGB camera connected to the Raspberry Pi (/dev/video0)
and transmits the image matrix over the network with near-zero latency.

Zero-Lag Optimizations:
  * Hardware V4L2 queue drained continuously via background capture thread.
  * CAP_PROP_BUFFERSIZE forced to 1 (eliminates multi-frame driver lag).
  * TCP_NODELAY enabled (disables Nagle's algorithm & packet coalescing).
  * Fast, optimized JPEG encoding (quality=70, 2x faster than 85).

Usage Examples:
  # Stream live camera frames continuously to laptop (Redmi 10 / LAN)
  python3 camera_streamer.py --target-ip 192.168.1.10 --stream

  # Single verified snapshot
  python3 camera_streamer.py --target-ip 192.168.1.10 --snapshot
"""

import sys
import os
import time
import json
import struct
import socket
import argparse
import threading
import subprocess
from typing import Optional, Tuple

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
    return "192.168.1.10"


def init_camera(device_index=None, width: int = 640, height: int = 480):
    candidates = []
    # Prefer persistent udev symlinks first if they exist
    for sym in ["/dev/camera", "/dev/drone_camera", "/dev/video-cam"]:
        if os.path.exists(sym) and sym not in candidates:
            candidates.append(sym)

    if device_index is not None and device_index not in candidates:
        candidates.append(device_index)

    for dev_num in [0, 1, 2, 3]:
        if dev_num not in candidates:
            candidates.append(dev_num)

    cap = None
    for dev in candidates:
        dev_desc = dev if isinstance(dev, str) else f"/dev/video{dev}"
        cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap = cv2.VideoCapture(dev)

        if cap.isOpened():
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            cap.set(cv2.CAP_PROP_FPS, 30)

            # Test warmup
            ok = False
            for _ in range(5):
                ret, _ = cap.read()
                if ret:
                    ok = True
                time.sleep(0.03)
            if ok:
                print(f"[+] Successfully opened camera device: {dev_desc}")
                return cap
            cap.release()

    return None


class CameraCaptureThread:
    """
    Dedicated background thread that continuously drains the V4L2 kernel ring buffer.
    Detects hardware disconnects, clears stale frames immediately (no freeze),
    and automatically re-opens the camera when re-plugged into any port.
    """
    def __init__(self, device=None, width: int = 640, height: int = 480):
        self.device = device
        self.width = width
        self.height = height
        self.cap = None
        self.frame = None
        self.is_new = False
        self.lock = threading.Lock()
        self.running = True

        # Initial attempt
        self.cap = init_camera(self.device, self.width, self.height)
        self.thread = threading.Thread(target=self._worker, daemon=True, name="CamGrabber")
        self.thread.start()

    def _worker(self):
        fail_count = 0
        fail_attempts = 0
        while self.running:
            # Recovery mode: if camera is disconnected or not opened
            if self.cap is None or not self.cap.isOpened():
                time.sleep(0.5)
                fail_attempts += 1

                # If camera is missing for > 3s, reset Raspberry Pi USB controller to clear error -71
                if fail_attempts % 6 == 0:
                    try:
                        print("\n[!] Camera device missing. Power-cycling Raspberry Pi USB hub to clear enumeration locks...")
                        subprocess.run(
                            "echo '0000:01:00.0' | sudo tee /sys/bus/pci/drivers/xhci_hcd/unbind >/dev/null && sleep 0.5 && echo '0000:01:00.0' | sudo tee /sys/bus/pci/drivers/xhci_hcd/bind >/dev/null",
                            shell=True, timeout=3
                        )
                        time.sleep(1.0)
                    except Exception as e:
                        pass

                new_cap = init_camera(self.device, self.width, self.height)
                if new_cap is not None:
                    print("\n[+] Camera hardware reconnected and opened successfully!")
                    self.cap = new_cap
                    fail_count = 0
                    fail_attempts = 0
                continue

            try:
                ret, frame = self.cap.read()
            except Exception:
                ret, frame = False, None

            if ret and frame is not None:
                fail_count = 0
                with self.lock:
                    self.frame = frame
                    self.is_new = True
            else:
                fail_count += 1
                if fail_count >= 8:
                    # 8 consecutive failed reads = Hardware disconnected / unplugged
                    print("\n[!] Camera device read failed. Hardware disconnected? Attempting recovery...")
                    with self.lock:
                        self.frame = None
                        self.is_new = False
                    try:
                        self.cap.release()
                    except Exception:
                        pass
                    self.cap = None
                    fail_count = 0
                time.sleep(0.01)

    def read(self) -> Tuple[bool, Optional[np.ndarray]]:
        with self.lock:
            if self.frame is not None and self.is_new:
                self.is_new = False  # Mark consumed so stale/frozen frames are never repeated
                return True, self.frame
            return False, None

    def stop(self):
        self.running = False
        if self.cap:
            try:
                self.cap.release()
            except Exception:
                pass


def send_frame(sock: socket.socket, frame: np.ndarray, frame_id: int, mode: str = "jpeg", jpeg_quality: int = 70) -> bool:
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
        print(f"\n[!] Socket transmission error: {e}")
        return False


def run_streamer(
    target_ip: str,
    port: int = 5000,
    device=None,
    mode: str = "jpeg",
    snapshot: bool = False,
    fps_limit: float = 30.0,
    jpeg_quality: int = 70
):
    if isinstance(device, str) and device.isdigit():
        device = int(device)

    dev_str = f"/dev/video{device}" if isinstance(device, int) else (str(device) if device else "/dev/camera (auto)")
    print("=" * 68)
    print(" RASPBERRY PI ZERO-LAG RGB CAMERA STREAMER")
    print("=" * 68)
    print(f"Target Laptop:    {target_ip}:{port}")
    print(f"Camera Device:    {dev_str}")
    print(f"Encoding Mode:    {mode.upper()} (quality={jpeg_quality})")
    print(f"Transmission:     {'SINGLE SNAPSHOT' if snapshot else 'CONTINUOUS ZERO-LAG STREAM'}")
    print(f"FPS Target:       {fps_limit}")
    print("=" * 68)

    # Initialize resilient background grabber thread with auto-hotplug recovery
    grabber = CameraCaptureThread(device=device, width=640, height=480)
    time.sleep(0.3)

    interval = 1.0 / max(fps_limit, 1.0)

    try:
        while True:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)

            connected = False
            first_attempt = True
            while not connected:
                try:
                    sock.connect((target_ip, port))
                    connected = True
                    print(f"\n[+] TCP connection established with Ground Station at {target_ip}:{port}!")
                except Exception as e:
                    if snapshot:
                        print(f"[FAILED] Could not connect to {target_ip}:{port}: {e}")
                        grabber.stop()
                        cap.release()
                        sock.close()
                        sys.exit(1)
                    if first_attempt:
                        print(f"Waiting for Ground Station receiver at {target_ip}:{port} (retrying every 1s)...")
                        first_attempt = False
                    time.sleep(1.0)

            frame_id = 0
            t_start = time.time()

            try:
                if snapshot:
                    ret, frame = grabber.read()
                    if ret and frame is not None:
                        ok = send_frame(sock, frame, frame_id=1, mode=mode, jpeg_quality=jpeg_quality)
                        if ok:
                            print(f"[SUCCESS] Sent image matrix snapshot: shape={frame.shape}, size={len(frame.tobytes())} bytes")
                        else:
                            print("[FAILED] Failed sending snapshot.")
                    break
                else:
                    print("Streaming live real-time frames... Press Ctrl+C to stop.")
                    while True:
                        loop_start = time.time()
                        ret, frame = grabber.read()
                        if not ret or frame is None:
                            time.sleep(0.005)
                            continue

                        frame_id += 1
                        ok = send_frame(sock, frame, frame_id=frame_id, mode=mode, jpeg_quality=jpeg_quality)
                        if not ok:
                            print("\n[!] Connection to Ground Station lost. Auto-reconnecting...")
                            break

                        elapsed = time.time() - t_start
                        rate = frame_id / elapsed if elapsed > 0 else 0
                        sys.stdout.write(f"\r[STREAMING] Frame {frame_id:>4} | Shape: {frame.shape} | Rate: {rate:>4.1f} FPS | Mean: {frame.mean():>5.1f}")
                        sys.stdout.flush()

                        delay = interval - (time.time() - loop_start)
                        if delay > 0:
                            time.sleep(delay)
            finally:
                sock.close()

    except KeyboardInterrupt:
        print("\nStopping camera streamer...")
    finally:
        grabber.stop()
        print("Camera streamer closed.")


def main():
    default_ip = get_default_target_ip()
    parser = argparse.ArgumentParser(description="Zero-Lag RGB Camera Matrix Streamer")
    parser.add_argument("--target-ip", type=str, default=default_ip, help=f"Target laptop IP (default: {default_ip})")
    parser.add_argument("--port", type=int, default=5000, help="TCP port (default: 5000)")
    parser.add_argument("--device", type=str, default="/dev/camera", help="V4L2 camera device path or index (default: /dev/camera)")
    parser.add_argument("--mode", type=str, choices=["jpeg", "raw"], default="jpeg", help="Matrix encoding mode (default: jpeg)")
    parser.add_argument("--quality", type=int, default=70, help="JPEG quality 1-100 (default: 70 for lowest latency)")
    parser.add_argument("--snapshot", action="store_true", help="Send single verified frame snapshot and exit")
    parser.add_argument("--stream", action="store_true", default=True, help="Stream frames continuously (default: True)")
    parser.add_argument("--fps", type=float, default=30.0, help="Maximum stream FPS limit (default: 30.0)")

    args = parser.parse_args()

    is_snapshot = args.snapshot
    run_streamer(
        target_ip=args.target_ip,
        port=args.port,
        device=args.device,
        mode=args.mode,
        snapshot=is_snapshot,
        fps_limit=args.fps,
        jpeg_quality=args.quality
    )


if __name__ == "__main__":
    main()
