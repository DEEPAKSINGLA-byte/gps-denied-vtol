#!/usr/bin/env python3
"""
segmentation_pipeline.py - Real-Time Segmentation Engine for Drone Camera Stream
================================================================================
Receives live RGB image matrices (H, W, 3) from the Raspberry Pi companion computer
and executes segmentation inference using Ultralytics YOLOv8 / PyTorch on the laptop.

Features:
  * Hardware acceleration on NVIDIA GPU (RTX 4060 Laptop GPU via CUDA)
  * Extracts class masks (N, H, W), polygon contours, bounding boxes & labels
  * Generates both annotated overlay and background-isolated mask frames
  * Turnkey programmatic API (importable in ROS 2 navigation & perception nodes)
"""

import sys
import time
import os
from typing import Optional, Dict, Any, Tuple, List

try:
    import cv2
    import numpy as np
    import torch
    from ultralytics import YOLO
except ImportError as e:
    print(f"ERROR: Missing dependency ({e}). Install via: pip install ultralytics opencv-python torch")
    sys.exit(1)


class SegmentationEngine:
    """
    High-performance segmentation processor for image matrices.
    """

    def __init__(
        self,
        model_path: str = "yolov8n-seg.pt",
        device: Optional[str] = None,
        conf_threshold: float = 0.25,
        iou_threshold: float = 0.45
    ):
        # Auto-select CUDA if available
        if device is None:
            self.device = "cuda:0" if torch.cuda.is_available() else "cpu"
        else:
            self.device = device

        print(f"Loading segmentation model '{model_path}' on device: {self.device.upper()}...")
        self.model = YOLO(model_path)
        self.conf = conf_threshold
        self.iou = iou_threshold

        # Warmup model with dummy frame
        dummy = np.zeros((480, 640, 3), dtype=np.uint8)
        _ = self.model(dummy, device=self.device, verbose=False)
        print(f"[+] Segmentation model ready! Device: {self.device.upper()}")

    def process_matrix(self, frame: np.ndarray) -> Dict[str, Any]:
        """
        Execute segmentation on an input image matrix.

        :param frame: NumPy array (H, W, 3) uint8 (BGR or RGB)
        :return: Structured dictionary containing masks, boxes, labels, and annotated frames.
        """
        if not isinstance(frame, np.ndarray) or frame.ndim != 3:
            raise ValueError(f"Expected 3D NumPy array (H, W, C), got: {type(frame)}")

        h, w, c = frame.shape
        t_start = time.time()

        # Run inference
        results = self.model(
            frame,
            device=self.device,
            conf=self.conf,
            iou=self.iou,
            verbose=False
        )
        t_infer = (time.time() - t_start) * 1000.0

        res = results[0]

        # 1. Annotated visualization overlay
        annotated_frame = res.plot()

        # 2. Extract binary and colored masks
        mask_overlay = np.zeros_like(frame)
        masks_data = []

        if res.masks is not None and len(res.masks) > 0:
            for i, mask_tensor in enumerate(res.masks.data):
                mask_np = mask_tensor.cpu().numpy()
                mask_resized = cv2.resize(mask_np, (w, h), interpolation=cv2.INTER_NEAREST)
                binary_mask = (mask_resized > 0.5).astype(np.uint8) * 255
                mask_overlay[mask_resized > 0.5] = frame[mask_resized > 0.5]

                cls_id = int(res.boxes.cls[i]) if res.boxes is not None else -1
                cls_name = self.model.names.get(cls_id, f"class_{cls_id}")
                conf = float(res.boxes.conf[i]) if res.boxes is not None else 0.0

                masks_data.append({
                    "class_id": cls_id,
                    "class_name": cls_name,
                    "confidence": round(conf, 3),
                    "binary_mask": binary_mask,
                    "pixel_area": int(np.sum(mask_resized > 0.5)),
                })

        # 3. Extract bounding boxes
        boxes_data = []
        if res.boxes is not None and len(res.boxes) > 0:
            for box in res.boxes:
                xyxy = box.xyxy[0].cpu().numpy().tolist()
                cls_id = int(box.cls[0])
                cls_name = self.model.names.get(cls_id, f"class_{cls_id}")
                conf = float(box.conf[0])
                boxes_data.append({
                    "class_id": cls_id,
                    "class_name": cls_name,
                    "confidence": round(conf, 3),
                    "bbox_xyxy": [round(x, 1) for x in xyxy],
                })

        return {
            "shape": (h, w, c),
            "inference_time_ms": round(t_infer, 2),
            "num_objects": len(boxes_data),
            "num_masks": len(masks_data),
            "detections": boxes_data,
            "masks": masks_data,
            "annotated_frame": annotated_frame,
            "isolated_mask_frame": mask_overlay,
        }


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Test segmentation on an image file")
    parser.add_argument("image", nargs="?", default="received_image_matrix.jpg", help="Path to input image")
    args = parser.parse_args()

    if not os.path.exists(args.image):
        print(f"Image '{args.image}' not found.")
        sys.exit(1)

    engine = SegmentationEngine()
    img = cv2.imread(args.image)
    out = engine.process_matrix(img)
    print(f"\nInference completed in {out['inference_time_ms']} ms")
    print(f"Detected: {out['num_objects']} objects, {out['num_masks']} segmentation masks")
    cv2.imwrite("segmented_output.jpg", out["annotated_frame"])
    print("Saved output to segmented_output.jpg")
