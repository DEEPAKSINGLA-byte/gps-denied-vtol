#!/usr/bin/env python3
"""
live_segmentation.py - Live Segmentation Receiver on Ground Laptop
==================================================================
Receives live RGB camera frames streamed from the Raspberry Pi over the Redmi 10 network,
executes real-time segmentation on the laptop GPU (NVIDIA RTX 4060 via CUDA),
and outputs annotated frames, masks, and detection metadata.

Usage Examples:
  # Verify pipeline on a single incoming frame from Pi
  python3 live_segmentation.py --verify

  # Run continuous live real-time segmentation (prints FPS & inference times)
  python3 live_segmentation.py --stream

  # Continuous segmentation with GUI display window
  python3 live_segmentation.py --stream --gui

  # Save annotated frames to output directory
  python3 live_segmentation.py --stream --save-dir ./segmentation_results
"""

import sys
import os
import time
import socket
import argparse
from typing import Optional

try:
    import cv2
    import numpy as np
    import torch
except ImportError as e:
    print(f"ERROR: Missing dependency ({e})")
    sys.exit(1)

from camera_receiver import read_frame
from segmentation_pipeline import SegmentationEngine


def verify_pipeline(port: int = 5000, model_path: str = "yolov8n-seg.pt", timeout: float = 15.0) -> bool:
    print("=" * 70)
    print(" LIVE SEGMENTATION PIPELINE VERIFICATION (REDMI 10 NETWORK)")
    print("=" * 70)

    # 1. Initialize Engine on GPU
    device = "cuda:0" if torch.cuda.is_available() else "cpu"
    print(f"Accelerating on: {device.upper()} ({torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'CPU'})")
    engine = SegmentationEngine(model_path=model_path, device=device)

    # 2. Listen for frame from Raspberry Pi
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind(("0.0.0.0", port))
    server_sock.listen(1)
    server_sock.settimeout(timeout)

    print(f"\nListening on 0.0.0.0:{port} for frame from Raspberry Pi...")
    try:
        client_sock, client_addr = server_sock.accept()
        print(f"[+] Connected to Raspberry Pi at {client_addr[0]}:{client_addr[1]}")
    except socket.timeout:
        print(f"[FAILED] Timed out waiting for connection on port {port}.")
        server_sock.close()
        return False

    client_sock.settimeout(10.0)
    res = read_frame(client_sock)
    client_sock.close()
    server_sock.close()

    if not res:
        print("[FAILED] Could not read frame from network stream.")
        return False

    metadata, frame, latency_ms = res
    print(f"[+] Received image matrix from Pi: shape={frame.shape}, dtype={frame.dtype}, transit={latency_ms:.1f}ms")

    # 3. Execute Segmentation on the Received Matrix
    print("\nExecuting segmentation inference on image matrix...")
    out = engine.process_matrix(frame)

    # 4. Save results to disk
    out_dir = os.path.dirname(os.path.abspath(__file__))
    annotated_path = os.path.join(out_dir, "segmented_result.jpg")
    mask_path = os.path.join(out_dir, "mask_overlay.jpg")

    cv2.imwrite(annotated_path, out["annotated_frame"])
    cv2.imwrite(mask_path, out["isolated_mask_frame"])

    # 5. Output Verification Report
    print("\n" + "=" * 70)
    print(" SEGMENTATION PIPELINE VALIDATION REPORT")
    print("=" * 70)
    print(f"Input Matrix Shape:      {out['shape']} (uint8)")
    print(f"Network Transit Latency: {latency_ms:.2f} ms")
    print(f"GPU Inference Time:      {out['inference_time_ms']:.2f} ms on {device.upper()}")
    print(f"Objects Detected:        {out['num_objects']}")
    print(f"Segmentation Masks:      {out['num_masks']}")

    for i, obj in enumerate(out["detections"]):
        print(f"  * Object {i+1}: {obj['class_name']} (conf: {obj['confidence']:.2f}, bbox: {obj['bbox_xyxy']})")

    print(f"Annotated Image Saved:   {annotated_path}")
    print(f"Isolated Mask Saved:     {mask_path}")
    print("=" * 70)
    print("\033[92m[SUCCESS] Image matrix successfully ingested and processed by segmentation model!\033[0m")
    print("=" * 70 + "\n")
    return True


def run_live_stream(
    port: int = 5000,
    model_path: str = "yolov8n-seg.pt",
    gui: bool = False,
    save_dir: Optional[str] = None
):
    device = "cuda:0" if torch.cuda.is_available() else "cpu"
    print(f"Initializing live segmentation on: {device.upper()}")
    engine = SegmentationEngine(model_path=model_path, device=device)

    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind(("0.0.0.0", port))
    server_sock.listen(1)

    if save_dir:
        os.makedirs(save_dir, exist_ok=True)

    print(f"Listening on 0.0.0.0:{port} for continuous camera stream...")
    try:
        client_sock, client_addr = server_sock.accept()
        print(f"[+] Connected to Raspberry Pi at {client_addr[0]}:{client_addr[1]}")

        frame_count = 0
        t_start = time.time()

        while True:
            res = read_frame(client_sock)
            if not res:
                print("\nStream ended by sender.")
                break

            meta, frame, latency = res
            frame_count += 1

            # Run Segmentation on GPU
            out = engine.process_matrix(frame)

            elapsed = time.time() - t_start
            overall_fps = frame_count / elapsed if elapsed > 0 else 0
            infer_ms = out["inference_time_ms"]
            n_obj = out["num_objects"]
            n_masks = out["num_masks"]

            sys.stdout.write(
                f"\r[SEGMENTING] Frame {frame_count:>4} | Shape: {frame.shape} | "
                f"GPU Infer: {infer_ms:>4.1f}ms | Pipeline: {overall_fps:>4.1f} FPS | "
                f"Transit: {latency:>4.1f}ms | Objects: {n_obj:>2} | Masks: {n_masks:>2}"
            )
            sys.stdout.flush()

            if save_dir and frame_count % 15 == 0:
                p_annot = os.path.join(save_dir, f"frame_{frame_count:05d}_seg.jpg")
                cv2.imwrite(p_annot, out["annotated_frame"])

            if gui:
                cv2.imshow("Live Segmentation Stream", out["annotated_frame"])
                cv2.imshow("Mask Overlay", out["isolated_mask_frame"])
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

    except KeyboardInterrupt:
        print("\nSegmentation stopped by user.")
    finally:
        server_sock.close()
        if gui:
            cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description="Live Drone Camera Stream Segmentation")
    parser.add_argument("--port", type=int, default=5000, help="TCP port (default: 5000)")
    parser.add_argument("--model", type=str, default="yolov8n-seg.pt", help="YOLOv8 segmentation weights (default: yolov8n-seg.pt)")
    parser.add_argument("--verify", action="store_true", help="Process 1 frame from Pi and generate verification report")
    parser.add_argument("--stream", action="store_true", help="Run continuous live segmentation stream")
    parser.add_argument("--gui", action="store_true", help="Display live OpenCV visualization window")
    parser.add_argument("--save-dir", type=str, default=None, help="Directory to save segmented outputs")

    args = parser.parse_args()

    if args.stream:
        run_live_stream(port=args.port, model_path=args.model, gui=args.gui, save_dir=args.save_dir)
    else:
        ok = verify_pipeline(port=args.port, model_path=args.model)
        sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
