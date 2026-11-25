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
  * Camera forced to MJPG: YUYV 640x480 saturates USB 2.0 and makes UVC cams
    drop to ~10-15 FPS with extra internal buffering.
  * --passthrough forwards the camera's own MJPEG bytes (no decode + re-encode on the Pi).
  * Frame dropping on congestion: TCP_NOTSENT_LOWAT keeps at most ~1 frame queued in the
    kernel; if the link is still busy when a new frame is ready, the frame is skipped
    instead of piling up behind the old ones (the main cause of multi-second lag on Wi-Fi).
  * Capture timestamp taken in the grabber thread, so measured latency is end-to-end.
  * DSCP AF41 marking -> Wi-Fi WMM "video" access category.

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
import select
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


def init_camera(device_index=None, width: int = 640, height: int = 480,
                passthrough: bool = False, fps: float = 30.0):
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
            # FOURCC must be set before the size, or the driver picks YUYV.
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            cap.set(cv2.CAP_PROP_FPS, fps)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if passthrough:
                cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)

            # Test warmup
            ok = False
            for _ in range(5):
                ret, _ = cap.read()
                if ret:
                    ok = True
                time.sleep(0.03)
            if ok:
                fcc = int(cap.get(cv2.CAP_PROP_FOURCC))
                fcc_s = "".join(chr((fcc >> (8 * i)) & 0xFF) for i in range(4))
                print(f"[+] Successfully opened camera device: {dev_desc} "
                      f"({int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))} "
                      f"{fcc_s} @ {cap.get(cv2.CAP_PROP_FPS):.0f} FPS)")
                if fcc_s != "MJPG":
                    print("[!] Camera did not accept MJPG; USB bandwidth may limit FPS/latency.")
                return cap
            cap.release()

    return None


class CameraCaptureThread:
    """
    Dedicated background thread that continuously drains the V4L2 kernel ring buffer.
    Detects hardware disconnects, clears stale frames immediately (no freeze),
    and automatically re-opens the camera when re-plugged into any port.
    """
    def __init__(self, device=None, width: int = 640, height: int = 480,
                 passthrough: bool = False, fps: float = 30.0, usb_reset: bool = False):
        self.device = device
        self.usb_reset = usb_reset
        self.width = width
        self.height = height
        self.passthrough = passthrough
        self.fps = fps
        self.cap = None
        self.frame = None
        self.stamp = 0.0
        self.shape = (height, width, 3)  # actual negotiated size, updated on (re)open
        self.is_new = False
        self.lock = threading.Lock()
        self.running = True

        # Initial attempt
        self.cap = self._open()
        self.thread = threading.Thread(target=self._worker, daemon=True, name="CamGrabber")
        self.thread.start()

    def _open(self):
        cap = init_camera(self.device, self.width, self.height, self.passthrough, self.fps)
        if cap is not None:
            w, h = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            if w > 0 and h > 0:
                self.shape = (h, w, 3)
        return cap

    def _worker(self):
        fail_count = 0
        fail_attempts = 0
        while self.running:
            # Recovery mode: if camera is disconnected or not opened
            if self.cap is None or not self.cap.isOpened():
                time.sleep(0.5)
                fail_attempts += 1

                # Opt-in (--usb-reset): if the camera is missing for > 3 s, unbind/rebind the
                # Pi 4 USB controller to clear UVC error -71. That controller also carries the
                # Pixhawk and the LiDAR, so this drops THEIR links too - never do it by default
                # (with the camera unplugged it would cut the flight-controller link every 3 s).
                if self.usb_reset and fail_attempts % 6 == 0:
                    try:
                        print("\n[!] Camera device missing. Power-cycling Raspberry Pi USB hub to clear enumeration locks...")
                        subprocess.run(
                            "echo '0000:01:00.0' | sudo -n tee /sys/bus/pci/drivers/xhci_hcd/unbind >/dev/null && sleep 0.5 && echo '0000:01:00.0' | sudo -n tee /sys/bus/pci/drivers/xhci_hcd/bind >/dev/null",
                            shell=True, timeout=3
                        )
                        time.sleep(1.0)
                    except Exception as e:
                        pass

                new_cap = self._open()
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
            stamp = time.time()  # wall clock: receivers compare it with their own clock

            if ret and frame is not None:
                fail_count = 0
                with self.lock:
                    self.frame = frame
                    self.stamp = stamp
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
        ok, frame, _ = self.read_stamped()
        return ok, frame

    def read_stamped(self) -> Tuple[bool, Optional[np.ndarray], float]:
        with self.lock:
            if self.frame is not None and self.is_new:
                self.is_new = False  # Mark consumed so stale/frozen frames are never repeated
                return True, self.frame, self.stamp
            return False, None, 0.0

    def stop(self):
        self.running = False
        if self.cap:
            try:
                self.cap.release()
            except Exception:
                pass


def _is_mjpeg_buffer(frame: np.ndarray) -> bool:
    """With CAP_PROP_CONVERT_RGB=0 + MJPG, OpenCV returns the raw JPEG bytes as a 1-row array."""
    if frame.dtype != np.uint8 or frame.ndim > 2 or (frame.ndim == 2 and frame.shape[0] != 1):
        return False
    flat = frame.reshape(-1)
    return flat.size > 4 and flat[0] == 0xFF and flat[1] == 0xD8


def send_frame(sock: socket.socket, frame: np.ndarray, frame_id: int, mode: str = "jpeg",
               jpeg_quality: int = 70, timestamp: Optional[float] = None,
               shape: Optional[Tuple[int, int, int]] = None) -> bool:
    """
    Serialize and transmit an image matrix with metadata header over TCP socket.
    ``timestamp`` should be the capture time; ``shape`` is required for MJPEG passthrough
    buffers (which carry no geometry of their own).
    """
    if timestamp is None:
        timestamp = time.time()
    mean_intensity = None

    if _is_mjpeg_buffer(frame):
        payload = frame.tobytes()
        encoding = "jpeg"
        h, w, c = shape if shape else (0, 0, 3)
    elif mode == "raw":
        h, w, c = frame.shape
        payload = frame.tobytes()
        encoding = "raw"
    else:
        h, w, c = frame.shape
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality]
        success, encimg = cv2.imencode(".jpg", frame, encode_param)
        if not success:
            return False
        payload = encimg.tobytes()
        encoding = "jpeg"
    if frame.ndim == 3:
        # Sparse sample: a full-frame mean costs several ms per frame on a Pi.
        mean_intensity = round(float(frame[::8, ::8].mean()), 2)

    metadata = {
        "frame_id": frame_id,
        "timestamp": timestamp,
        "shape": [h, w, c],
        "dtype": str(frame.dtype),
        "encoding": encoding,
        "payload_size": len(payload),
    }
    if mean_intensity is not None:
        metadata["mean_intensity"] = mean_intensity

    meta_bytes = json.dumps(metadata).encode("utf-8")
    header = struct.pack(">II", len(meta_bytes), len(payload))

    try:
        sock.sendall(header + meta_bytes + payload)
        return True
    except Exception as e:
        print(f"\n[!] Socket transmission error: {e}")
        return False


def _new_stream_socket() -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    # NOTSENT_LOWAT: the socket only reports writable once the previous frame has
    # (almost) left the host, so we can drop frames instead of queueing them.
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 131072)
    try:
        sock.setsockopt(socket.IPPROTO_TCP, getattr(socket, "TCP_NOTSENT_LOWAT", 25), 16384)
    except OSError:
        pass
    try:
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_TOS, 0x88)  # DSCP AF41 -> WMM video
    except OSError:
        pass
    try:
        # Abort the connection if sent data stays un-ACKed for 3 s (Wi-Fi gone). Without
        # this a silently dead link is only noticed after the kernel's ~15 min
        # retransmission timeout: select() never reports the socket writable again, so
        # every frame is "dropped" and the streamer never reconnects.
        sock.setsockopt(socket.IPPROTO_TCP, getattr(socket, "TCP_USER_TIMEOUT", 18), 3000)
    except OSError:
        pass
    return sock


def run_streamer(
    target_ip: str,
    port: int = 5000,
    device=None,
    mode: str = "jpeg",
    snapshot: bool = False,
    fps_limit: float = 30.0,
    jpeg_quality: int = 70,
    width: int = 640,
    height: int = 480,
    passthrough: bool = False,
    usb_reset: bool = False,
):
    if isinstance(device, str) and device.isdigit():
        device = int(device)

    dev_str = f"/dev/video{device}" if isinstance(device, int) else (str(device) if device else "/dev/camera (auto)")
    print("=" * 68)
    print(" RASPBERRY PI ZERO-LAG RGB CAMERA STREAMER")
    print("=" * 68)
    print(f"Target Laptop:    {target_ip}:{port}")
    print(f"Camera Device:    {dev_str}")
    print(f"Resolution:       {width}x{height}")
    print(f"Encoding Mode:    {'MJPEG PASSTHROUGH' if passthrough else f'{mode.upper()} (quality={jpeg_quality})'}")
    print(f"Transmission:     {'SINGLE SNAPSHOT' if snapshot else 'CONTINUOUS ZERO-LAG STREAM'}")
    print(f"FPS Target:       {fps_limit}")
    print("=" * 68)

    # Initialize resilient background grabber thread with auto-hotplug recovery
    grabber = CameraCaptureThread(device=device, width=width, height=height,
                                  passthrough=passthrough, fps=fps_limit, usb_reset=usb_reset)
    time.sleep(0.3)

    interval = 1.0 / max(fps_limit, 1.0)

    try:
        while True:
            connected = False
            first_attempt = True
            while not connected:
                # A fresh socket per attempt: after a timed-out connect() the old one is
                # still "connecting" and any retry on it fails with EALREADY.
                sock = _new_stream_socket()
                try:
                    sock.settimeout(3.0)          # don't hang ~2 min on an absent laptop
                    sock.connect((target_ip, port))
                    sock.settimeout(None)         # blocking sends (select() gates them)
                    connected = True
                    print(f"\n[+] TCP connection established with Ground Station at {target_ip}:{port}!")
                except Exception as e:
                    sock.close()
                    if snapshot:
                        print(f"[FAILED] Could not connect to {target_ip}:{port}: {e}")
                        sys.exit(1)  # outer finally stops the grabber
                    if first_attempt:
                        print(f"Waiting for Ground Station receiver at {target_ip}:{port} (retrying every 1s)...")
                        first_attempt = False
                    time.sleep(1.0)

            frame_id = 0
            dropped = 0
            t_start = time.time()

            try:
                if snapshot:
                    ret, frame, stamp = False, None, 0.0
                    for _ in range(200):  # grabber may not have a frame yet
                        ret, frame, stamp = grabber.read_stamped()
                        if ret:
                            break
                        time.sleep(0.01)
                    if ret and frame is not None:
                        ok = send_frame(sock, frame, frame_id=1, mode=mode, jpeg_quality=jpeg_quality,
                                        timestamp=stamp, shape=grabber.shape)
                        if ok:
                            print(f"[SUCCESS] Sent image matrix snapshot: shape={frame.shape}, size={len(frame.tobytes())} bytes")
                        else:
                            print("[FAILED] Failed sending snapshot.")
                    break
                else:
                    print("Streaming live real-time frames... Press Ctrl+C to stop.")
                    while True:
                        loop_start = time.time()
                        ret, frame, stamp = grabber.read_stamped()
                        if not ret or frame is None:
                            time.sleep(0.002)
                            continue

                        # Link still busy with the previous frame -> drop this one rather
                        # than queue it (queued frames are what turn into seconds of lag).
                        _, writable, _ = select.select([], [sock], [], 0)
                        if not writable:
                            dropped += 1
                            continue

                        frame_id += 1
                        ok = send_frame(sock, frame, frame_id=frame_id, mode=mode, jpeg_quality=jpeg_quality,
                                        timestamp=stamp, shape=grabber.shape)
                        if not ok:
                            print("\n[!] Connection to Ground Station lost. Auto-reconnecting...")
                            break

                        elapsed = time.time() - t_start
                        rate = frame_id / elapsed if elapsed > 0 else 0
                        age_ms = (time.time() - stamp) * 1e3
                        sys.stdout.write(f"\r[STREAMING] Frame {frame_id:>4} | Rate: {rate:>4.1f} FPS | "
                                         f"Dropped: {dropped:>4} | Capture->sent: {age_ms:>5.1f} ms")
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
    parser.add_argument("--width", type=int, default=640, help="Capture width (default: 640)")
    parser.add_argument("--height", type=int, default=480, help="Capture height (default: 480)")
    parser.add_argument("--usb-reset", action="store_true",
                        help="If the camera disappears, reset the Pi 4 USB controller to recover it. "
                             "WARNING: this also disconnects the Pixhawk and LiDAR if they are on USB")
    parser.add_argument("--passthrough", action="store_true",
                        help="Send the camera's own MJPEG frames untouched (lowest Pi CPU/latency; "
                             "--quality is ignored, frames may be larger)")

    args = parser.parse_args()

    is_snapshot = args.snapshot
    run_streamer(
        target_ip=args.target_ip,
        port=args.port,
        device=args.device,
        mode=args.mode,
        snapshot=is_snapshot,
        fps_limit=args.fps,
        jpeg_quality=args.quality,
        width=args.width,
        height=args.height,
        passthrough=args.passthrough,
        usb_reset=args.usb_reset,
    )


if __name__ == "__main__":
    main()
