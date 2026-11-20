#!/usr/bin/env python3
"""
segmentation_pipeline.py - Real-Time YOLO + SAM Segmentation Engine for Drone Camera Stream
===========================================================================================
Receives live RGB image matrices (H, W, 3) from the Raspberry Pi companion computer
and runs hybrid segmentation on the laptop GPU:

    YOLOv8-seg  -> classes, boxes, coarse masks          (~5 ms)
    SAM 2.1     -> sharp masks prompted by YOLO's boxes  (~50 ms @1024, ~30 ms @768)
    fusion      -> per instance, SAM mask if it passes quality gates, else YOLO mask

The model logic lives in ros2_ws/src/quadcopter_perception/quadcopter_perception/
hybrid_segmenter.py (no ROS dependency) and is shared with the ROS 2 segmentation_node,
so both paths produce identical masks.

Features:
  * Hardware acceleration on NVIDIA GPU via CUDA (FP16)
  * Extracts class masks (N, H, W), bounding boxes & labels, mask source per instance
  * Generates both annotated overlay and background-isolated mask frames
  * Turnkey programmatic API (importable in ROS 2 navigation & perception nodes)
  * sam_model=None -> plain YOLO (A/B comparison / lowest latency)
"""

import os
import sys
from typing import Any, Dict, Optional

try:
    import cv2
    import numpy as np
except ImportError as e:
    print(f"ERROR: Missing dependency ({e}). Install via: pip install -U ultralytics opencv-python torch")
    sys.exit(1)

# Shared core lives in the ROS 2 package; import it straight from the source tree so this
# script works without sourcing/building the ROS workspace.
_PKG_DIR = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "ros2_ws", "src", "quadcopter_perception"))
try:
    from quadcopter_perception.hybrid_segmenter import HybridSegmenter, SegmentationResult
except ImportError:
    sys.path.insert(0, _PKG_DIR)
    from quadcopter_perception.hybrid_segmenter import HybridSegmenter, SegmentationResult

DEFAULT_SAM_MODEL = "sam2.1_t.pt"


class SegmentationEngine:
    """
    High-performance YOLO + SAM segmentation processor for image matrices.
    """

    def __init__(
        self,
        model_path: str = "yolov8n-seg.pt",
        device: Optional[str] = None,
        conf_threshold: float = 0.25,
        iou_threshold: float = 0.45,
        sam_model: Optional[str] = DEFAULT_SAM_MODEL,
        sam_imgsz: int = 1024,
        **hybrid_kwargs,
    ):
        """
        :param model_path: YOLO weights (segment or detect model)
        :param device:     'cuda:0', 'cpu', or None for auto
        :param sam_model:  SAM weights ('sam2.1_t.pt', 'sam2.1_s.pt', 'mobile_sam.pt', ...)
                           or None to disable SAM refinement
        :param sam_imgsz:  SAM input size (1024 best quality, 768 ~2x faster)
        :param hybrid_kwargs: extra HybridSegmenter options (sam_max_boxes, sam_min_score, ...)
        """
        print(f"Loading segmentation models (YOLO '{model_path}', SAM '{sam_model or 'off'}')...")
        self.seg = HybridSegmenter(
            yolo_model=model_path,
            sam_model=sam_model,
            device=device or "",
            conf=conf_threshold,
            iou=iou_threshold,
            sam_imgsz=sam_imgsz,
            **hybrid_kwargs,
        )
        self.device = self.seg.device
        self.names = self.seg.names
        print(f"[+] Segmentation models ready! Device: {self.device.upper()} | "
              f"SAM refinement: {'ON (' + self.seg.sam_model + ')' if self.seg.sam_enabled else 'OFF'}")

    def process_matrix(self, frame: np.ndarray, refine: bool = True) -> Dict[str, Any]:
        """
        Execute segmentation on an input image matrix.

        :param frame:  NumPy array (H, W, 3) uint8, BGR (OpenCV order)
        :param refine: False skips SAM for this frame (YOLO masks only)
        :return: Structured dictionary containing masks, boxes, labels, and annotated frames.
        """
        if not isinstance(frame, np.ndarray) or frame.ndim != 3:
            raise ValueError(f"Expected 3D NumPy array (H, W, C), got: {type(frame)}")

        h, w, c = frame.shape
        res: SegmentationResult = self.seg.segment(frame, refine=refine)

        mask_overlay = np.zeros_like(frame)
        masks_data, boxes_data = [], []
        for inst in res.instances:
            boxes_data.append({
                "class_id": inst.class_id,
                "class_name": inst.class_name,
                "confidence": round(inst.score, 3),
                "bbox_xyxy": [round(float(x), 1) for x in inst.box],
                "mask_source": inst.mask_source,
            })
            if inst.mask.any():
                mask_overlay[inst.mask] = frame[inst.mask]
                masks_data.append({
                    "class_id": inst.class_id,
                    "class_name": inst.class_name,
                    "confidence": round(inst.score, 3),
                    "binary_mask": inst.mask.astype(np.uint8) * 255,
                    "pixel_area": int(np.count_nonzero(inst.mask)),
                    "mask_source": inst.mask_source,          # 'sam' | 'yolo'
                    "sam_score": round(inst.sam_score, 3),    # nan if SAM not run
                })

        t = res.timing_ms
        return {
            "shape": (h, w, c),
            "inference_time_ms": round(t["total"], 2),
            "yolo_time_ms": round(t["yolo"], 2),
            "sam_time_ms": round(t["sam"], 2),
            "num_objects": len(boxes_data),
            "num_masks": len(masks_data),
            "num_sam_refined": res.num_sam_refined,
            "detections": boxes_data,
            "masks": masks_data,
            "class_mask": res.class_mask,          # uint8 HxW, class_id + 1 (0 = bg)
            "instance_mask": res.instance_mask,    # uint8 HxW, index + 1 (0 = bg)
            "annotated_frame": self.seg.annotate(frame, res),
            "isolated_mask_frame": mask_overlay,
        }


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Test YOLO + SAM segmentation on an image file")
    parser.add_argument("image", nargs="?", default="received_image_matrix.jpg", help="Path to input image")
    parser.add_argument("--model", default="yolov8n-seg.pt", help="YOLO weights")
    parser.add_argument("--sam-model", default=DEFAULT_SAM_MODEL, help="SAM weights, or 'none' to disable")
    parser.add_argument("--sam-imgsz", type=int, default=1024, help="SAM input size (768 = faster)")
    parser.add_argument("--out", default="segmented_output.jpg", help="Output image path")
    args = parser.parse_args()

    if not os.path.exists(args.image):
        print(f"Image '{args.image}' not found.")
        sys.exit(1)

    engine = SegmentationEngine(model_path=args.model, sam_model=args.sam_model,
                                sam_imgsz=args.sam_imgsz)
    img = cv2.imread(args.image)
    out = engine.process_matrix(img)
    print(f"\nInference completed in {out['inference_time_ms']} ms "
          f"(YOLO {out['yolo_time_ms']} ms + SAM {out['sam_time_ms']} ms)")
    print(f"Detected: {out['num_objects']} objects, {out['num_masks']} masks "
          f"({out['num_sam_refined']} SAM-refined)")
    for d in out["detections"]:
        print(f"  * {d['class_name']:<14} conf {d['confidence']:.2f}  mask: {d['mask_source']}")
    cv2.imwrite(args.out, out["annotated_frame"])
    print(f"Saved output to {args.out}")
