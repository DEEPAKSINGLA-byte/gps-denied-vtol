#!/usr/bin/env python3
"""
hybrid_segmenter.py - YOLO detection + SAM mask refinement (ROS-independent core).

    frame ──> YOLO-seg ──> boxes, classes, scores, coarse masks
                 │
                 └──> boxes as prompts ──> SAM (one image encode per frame)
                                             │
                                             v
                          fuse_masks(): per instance, keep the SAM mask only if it
                          passes quality gates, otherwise fall back to the YOLO mask

Why both:
  * YOLO gives the *semantics* (what / where / how confident) at a few ms per frame, but
    its masks are coarse: they come from a 160x120 prototype grid, so they are blobby at
    object boundaries and miss thin parts.
  * SAM gives class-agnostic, sharp boundaries, but needs a prompt and knows nothing
    about classes. Prompting SAM with YOLO's boxes gets the best of both.

Gates applied per instance before a SAM mask replaces the YOLO one (see fuse_masks):
  1. SAM's own predicted mask quality (IoU head)  >= sam_min_score
  2. the SAM mask is clipped to the YOLO box grown by box_margin (stops leaking into
     neighbouring objects / the background)
  3. SAM mask covers >= min_fill of the box (rejects degenerate slivers)
  4. IoU(SAM, YOLO mask) >= min_agreement, when a YOLO mask exists (rejects SAM picking
     a different object or a sub-part, e.g. a shirt instead of the person)

Usage
-----
    seg = HybridSegmenter(yolo_model='yolov8n-seg.pt', sam_model='sam2.1_t.pt')
    out = seg.segment(frame_bgr)           # SegmentationResult
    out.class_mask, out.instance_mask      # uint8 HxW (0 = background)
    vis = seg.annotate(frame_bgr, out)

Set sam_model='' (or None) to run plain YOLO, e.g. for A/B comparisons.
Only depends on numpy / opencv / torch / ultralytics - no ROS - so it is shared by the
ROS 2 node (segmentation_node.py) and the standalone raspberry_pi/ scripts.
"""

from __future__ import annotations

import os
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence

import cv2
import numpy as np

MASK_SOURCE_SAM = 'sam'
MASK_SOURCE_YOLO = 'yolo'
MASK_SOURCE_NONE = 'none'     # detect-only YOLO model and SAM rejected / disabled

# sam_model values meaning "no SAM" (empty strings are awkward to pass via ros2 launch)
SAM_DISABLED_VALUES = ('', "''", '""', 'none', 'off', 'false', '0')


# ============================================================================ data
@dataclass
class Instance:
    class_id: int
    class_name: str
    score: float                  # YOLO detection confidence
    box: np.ndarray               # (4,) float xyxy, pixels
    mask: np.ndarray              # (H, W) bool, final fused mask
    mask_source: str              # MASK_SOURCE_*
    sam_score: float = float('nan')       # SAM predicted mask IoU (nan = SAM not run)
    sam_agreement: float = float('nan')   # IoU(SAM mask, YOLO mask) (nan = n/a)


@dataclass
class SegmentationResult:
    instances: List[Instance]
    class_mask: np.ndarray        # (H, W) uint8, class_id + 1 (0 = background)
    instance_mask: np.ndarray     # (H, W) uint8, instance index + 1 (0 = background)
    timing_ms: Dict[str, float] = field(default_factory=dict)   # yolo / sam / fuse / total

    @property
    def num_sam_refined(self) -> int:
        return sum(i.mask_source == MASK_SOURCE_SAM for i in self.instances)


# ======================================================================= pure logic
def mask_iou(a: np.ndarray, b: np.ndarray) -> float:
    """IoU of two boolean masks (0.0 when both are empty)."""
    union = np.count_nonzero(a | b)
    return float(np.count_nonzero(a & b)) / union if union else 0.0


def expand_box(box: Sequence[float], margin: float, w: int, h: int) -> tuple:
    """Grow an xyxy box by `margin` x its size on each side (min 2 px), clip to image,
    return integer (x1, y1, x2, y2) usable as a slice (x2/y2 exclusive)."""
    x1, y1, x2, y2 = (float(v) for v in box)
    mx = max(2.0, margin * (x2 - x1))
    my = max(2.0, margin * (y2 - y1))
    return (max(0, int(np.floor(x1 - mx))), max(0, int(np.floor(y1 - my))),
            min(w, int(np.ceil(x2 + mx))), min(h, int(np.ceil(y2 + my))))


def fuse_masks(boxes: np.ndarray,
               yolo_masks: Optional[np.ndarray],
               sam_masks: Optional[np.ndarray],
               sam_scores: Optional[np.ndarray],
               refined: np.ndarray,
               *,
               min_score: float = 0.5,
               min_agreement: float = 0.3,
               min_fill: float = 0.05,
               box_margin: float = 0.1):
    """Decide, per instance, which mask to keep.

    Args:
        boxes:       (N, 4) xyxy pixel boxes from YOLO.
        yolo_masks:  (N, H, W) bool, or None for a detect-only YOLO model.
        sam_masks:   (M, H, W) bool masks for the instances in `refined`, or None.
        sam_scores:  (M,) SAM predicted-IoU scores, or None.
        refined:     (M,) int indices into boxes that SAM was prompted with, same order
                     as sam_masks.

    Returns:
        masks   (N, H, W) bool  fused masks
        sources list[str]       MASK_SOURCE_* per instance
        scores  (N,) float      SAM score per instance (nan where SAM was not run)
        agree   (N,) float      IoU(SAM, YOLO) per instance (nan where n/a)
    """
    n = len(boxes)
    if yolo_masks is not None:
        h, w = yolo_masks.shape[1:]
    elif sam_masks is not None and len(sam_masks):
        h, w = sam_masks.shape[1:]
    else:
        raise ValueError('fuse_masks needs yolo_masks or sam_masks to know the frame size')

    if yolo_masks is not None:
        out = yolo_masks.copy()
        sources = [MASK_SOURCE_YOLO] * n
    else:
        out = np.zeros((n, h, w), bool)
        sources = [MASK_SOURCE_NONE] * n
    scores = np.full(n, np.nan, np.float32)
    agree = np.full(n, np.nan, np.float32)

    if sam_masks is None or len(refined) == 0:
        return out, sources, scores, agree

    for j, i in enumerate(refined):
        i = int(i)
        scores[i] = float(sam_scores[j]) if sam_scores is not None else np.nan

        # Gate 2: clip SAM to the (slightly grown) detector box
        x1, y1, x2, y2 = expand_box(boxes[i], box_margin, w, h)
        sam = np.zeros((h, w), bool)
        sam[y1:y2, x1:x2] = sam_masks[j][y1:y2, x1:x2]

        if yolo_masks is not None:
            agree[i] = mask_iou(sam, yolo_masks[i])

        # Gate 1: SAM's own quality estimate
        if not scores[i] >= min_score:            # also rejects nan
            continue
        # Gate 3: degenerate / empty mask
        bx1, by1, bx2, by2 = boxes[i]
        box_area = max(1.0, float(bx2 - bx1) * float(by2 - by1))
        if np.count_nonzero(sam) < min_fill * box_area:
            continue
        # Gate 4: SAM and YOLO must be segmenting the same object
        if yolo_masks is not None and agree[i] < min_agreement:
            continue

        out[i] = sam
        sources[i] = MASK_SOURCE_SAM

    return out, sources, scores, agree


def rasterize(masks: np.ndarray, class_ids: np.ndarray, scores: np.ndarray,
              shape: tuple) -> tuple:
    """Paint instance masks into class / instance label images.

    Painted largest -> smallest mask (ties: low -> high confidence), so where masks
    overlap the smaller object wins. Small objects overlapping big ones are almost always
    in front of / attached to them (a tie on a person, a phone in a hand); painting by
    confidence alone would erase them from the label images.
    class pixel = class_id + 1, instance pixel = index + 1 (capped at 255); 0 = background.
    """
    class_mask = np.zeros(shape, np.uint8)
    inst_mask = np.zeros(shape, np.uint8)
    areas = masks.reshape(len(masks), -1).sum(1) if len(masks) else np.zeros(0)
    for i in np.lexsort((np.asarray(scores), -areas)):    # primary: area desc
        m = masks[i]
        class_mask[m] = min(int(class_ids[i]) + 1, 255)
        inst_mask[m] = min(int(i) + 1, 255)
    return class_mask, inst_mask


def _is_sam1_family(name: str) -> bool:
    """SAM v1 / MobileSAM weights (vs SAM 2 / 2.1). They produce EMPTY masks in FP16
    through Ultralytics (measured with mobile_sam.pt), so they must run in FP32."""
    base = os.path.basename(str(name)).lower()
    return 'sam2' not in base and 'sam3' not in base


# ======================================================================== engine
class HybridSegmenter:
    """YOLO (detect / seg) + SAM box-prompted mask refinement."""

    def __init__(self,
                 yolo_model: str = 'yolov8n-seg.pt',
                 sam_model: Optional[str] = 'sam2.1_t.pt',
                 device: str = '',
                 conf: float = 0.35,
                 iou: float = 0.45,
                 imgsz: int = 640,
                 half: bool = True,
                 classes: Optional[Sequence[str]] = None,
                 sam_imgsz: int = 1024,
                 sam_max_boxes: int = 8,
                 sam_min_box_px: int = 16,
                 sam_min_score: float = 0.5,
                 sam_min_agreement: float = 0.3,
                 sam_min_fill: float = 0.05,
                 sam_box_margin: float = 0.1,
                 warmup: bool = True,
                 logger=None):
        import torch                       # heavy imports deferred to construction
        from ultralytics import YOLO

        self._log = logger or (lambda msg: print(f'[hybrid_segmenter] {msg}'))
        self.device = device or ('cuda:0' if torch.cuda.is_available() else 'cpu')
        on_cuda = self.device.startswith('cuda')
        self.half = bool(half) and on_cuda
        self.conf, self.iou, self.imgsz = float(conf), float(iou), int(imgsz)

        self._log(f'Loading YOLO {yolo_model} on {self.device} (half={self.half})')
        self.yolo = YOLO(str(yolo_model))
        self.names: Dict[int, str] = {int(k): v for k, v in self.yolo.names.items()}
        self.yolo_has_masks = self.yolo.task == 'segment'

        wanted = [c for c in (classes or []) if c]
        unknown = [c for c in wanted if c not in self.names.values()]
        if unknown:
            self._log(f'Unknown class names ignored: {unknown}')
        ids = [i for i, n in self.names.items() if n in wanted]
        self.class_ids = ids or None       # None = all classes

        # SAM settings
        self.sam = None
        self.sam_model = str(sam_model or '').strip()
        if self.sam_model.lower() in SAM_DISABLED_VALUES:
            self.sam_model = ''
        self.sam_imgsz = int(sam_imgsz)
        self.sam_max_boxes = int(sam_max_boxes)
        self.sam_min_box_px = int(sam_min_box_px)
        self.fuse_kw = dict(min_score=float(sam_min_score),
                            min_agreement=float(sam_min_agreement),
                            min_fill=float(sam_min_fill),
                            box_margin=float(sam_box_margin))
        if self.sam_model:
            from ultralytics import SAM
            self.sam_half = self.half and not _is_sam1_family(self.sam_model)
            if self.half and not self.sam_half:
                self._log(f'{self.sam_model} is SAM v1-family: running it in FP32 '
                          '(FP16 yields empty masks)')
            self._log(f'Loading SAM {self.sam_model} (imgsz={self.sam_imgsz}, '
                      f'half={self.sam_half})')
            self.sam = SAM(self.sam_model)
        elif not self.yolo_has_masks:
            self._log('WARNING: detect-only YOLO model and SAM disabled -> no masks')

        if warmup:   # first call pays CUDA / cuDNN init + SAM model build (~1-3 s)
            dummy = np.zeros((480, 640, 3), np.uint8)
            self._yolo(dummy)
            if self.sam is not None:
                self._sam(dummy, np.array([[200.0, 150.0, 440.0, 330.0]]))

    # ---------------------------------------------------------------- model calls
    def _yolo(self, frame):
        return self.yolo.predict(frame, device=self.device, half=self.half, conf=self.conf,
                                 iou=self.iou, imgsz=self.imgsz, classes=self.class_ids,
                                 retina_masks=True, verbose=False)[0]

    def _sam(self, frame, boxes: np.ndarray):
        """Returns (masks (M,H,W) bool, scores (M,)) aligned 1:1 with `boxes`.

        conf=0.0 is essential: Ultralytics' SAM postprocess silently DROPS masks whose
        predicted IoU is below `conf` (default 0.25), which would misalign masks with the
        prompting boxes. Gating is done in fuse_masks instead.
        """
        res = self.sam.predict(frame, bboxes=boxes, device=self.device, half=self.sam_half,
                               imgsz=self.sam_imgsz, conf=0.0, verbose=False)[0]
        if res.masks is None or len(res.masks) != len(boxes):
            got = 0 if res.masks is None else len(res.masks)
            raise RuntimeError(f'SAM returned {got} masks for {len(boxes)} boxes')
        masks = res.masks.data.cpu().numpy().astype(bool)
        scores = res.boxes.conf.float().cpu().numpy()
        return masks, scores

    # -------------------------------------------------------------------- public
    @property
    def sam_enabled(self) -> bool:
        return self.sam is not None

    def segment(self, frame: np.ndarray, refine: bool = True) -> SegmentationResult:
        """Segment one BGR uint8 frame. `refine=False` skips SAM for this frame."""
        if not isinstance(frame, np.ndarray) or frame.ndim != 3 or frame.shape[2] != 3:
            got = getattr(frame, 'shape', type(frame))
            raise ValueError(f'expected (H, W, 3) uint8 image, got {got}')
        h, w = frame.shape[:2]
        timing = {'yolo': 0.0, 'sam': 0.0, 'fuse': 0.0}
        t_start = time.perf_counter()

        res = self._yolo(frame)
        timing['yolo'] = (time.perf_counter() - t_start) * 1e3

        boxes = res.boxes
        n = 0 if boxes is None else len(boxes)
        cls = boxes.cls.int().cpu().numpy() if n else np.zeros(0, int)
        conf = boxes.conf.float().cpu().numpy() if n else np.zeros(0, np.float32)
        xyxy = boxes.xyxy.float().cpu().numpy() if n else np.zeros((0, 4), np.float32)

        yolo_masks = None
        if n and res.masks is not None:
            yolo_masks = res.masks.data.cpu().numpy() > 0.5        # (n, h, w) retina
            if yolo_masks.shape[1:] != (h, w):                     # defensive
                yolo_masks = np.stack([cv2.resize(m.astype(np.uint8), (w, h),
                                                  interpolation=cv2.INTER_NEAREST) > 0
                                       for m in yolo_masks])

        # Pick which detections SAM refines: most confident first, skip tiny boxes
        # (SAM is unreliable below ~16 px and the YOLO mask is as good there).
        refined = np.zeros(0, int)
        sam_masks = sam_scores = None
        if n and refine and self.sam is not None:
            bw, bh = xyxy[:, 2] - xyxy[:, 0], xyxy[:, 3] - xyxy[:, 1]
            ok = np.flatnonzero(np.minimum(bw, bh) >= self.sam_min_box_px)
            refined = ok[np.argsort(-conf[ok], kind='stable')][:self.sam_max_boxes]
            if len(refined):
                t0 = time.perf_counter()
                try:
                    sam_masks, sam_scores = self._sam(frame, xyxy[refined])
                except Exception as e:     # never lose the frame because SAM failed
                    self._log(f'SAM failed, using YOLO masks for this frame: {e}')
                    refined = np.zeros(0, int)
                timing['sam'] = (time.perf_counter() - t0) * 1e3

        t0 = time.perf_counter()
        instances: List[Instance] = []
        if n:
            if yolo_masks is None and sam_masks is None:
                masks = np.zeros((n, h, w), bool)
                sources = [MASK_SOURCE_NONE] * n
                s_scores = agree = np.full(n, np.nan, np.float32)
            else:
                masks, sources, s_scores, agree = fuse_masks(
                    xyxy, yolo_masks, sam_masks, sam_scores, refined, **self.fuse_kw)
            for i in range(n):
                instances.append(Instance(
                    class_id=int(cls[i]),
                    class_name=self.names.get(int(cls[i]), str(int(cls[i]))),
                    score=float(conf[i]), box=xyxy[i], mask=masks[i],
                    mask_source=sources[i], sam_score=float(s_scores[i]),
                    sam_agreement=float(agree[i])))
            class_mask, inst_mask = rasterize(masks, cls, conf, (h, w))
        else:
            class_mask = np.zeros((h, w), np.uint8)
            inst_mask = np.zeros((h, w), np.uint8)
        timing['fuse'] = (time.perf_counter() - t0) * 1e3
        timing['total'] = (time.perf_counter() - t_start) * 1e3

        return SegmentationResult(instances, class_mask, inst_mask, timing)

    # ------------------------------------------------------------- visualisation
    @staticmethod
    def _color(class_id: int) -> tuple:
        """Stable, distinct BGR colour per class (golden-ratio hue walk)."""
        hue = int((class_id * 0.618033988749895 % 1.0) * 180)
        hsv = np.uint8([[[hue, 220, 255]]])
        return tuple(int(c) for c in cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0, 0])

    def annotate(self, frame: np.ndarray, result: SegmentationResult,
                 alpha: float = 0.45) -> np.ndarray:
        """Overlay fused masks, contours, boxes and labels.

        Label suffix shows where the mask came from: [S] = SAM-refined, [Y] = YOLO
        fallback, [-] = no mask.
        """
        out = frame.copy()
        if not result.instances:
            return out
        color_layer = out.copy()
        # same paint order as rasterize(): big first, so small objects stay visible
        areas = np.array([np.count_nonzero(inst.mask) for inst in result.instances])
        scores = np.array([inst.score for inst in result.instances])
        for i in np.lexsort((scores, -areas)):
            inst = result.instances[i]
            color_layer[inst.mask] = self._color(inst.class_id)
        cv2.addWeighted(color_layer, alpha, out, 1 - alpha, 0, out)

        tag = {MASK_SOURCE_SAM: 'S', MASK_SOURCE_YOLO: 'Y', MASK_SOURCE_NONE: '-'}
        for inst in result.instances:
            c = self._color(inst.class_id)
            if inst.mask.any():
                contours, _ = cv2.findContours(inst.mask.astype(np.uint8),
                                               cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(out, contours, -1, c, 2, cv2.LINE_AA)
            x1, y1, x2, y2 = (int(round(v)) for v in inst.box)
            cv2.rectangle(out, (x1, y1), (x2, y2), c, 1, cv2.LINE_AA)
            label = f'{inst.class_name} {inst.score:.2f} [{tag[inst.mask_source]}]'
            (tw, th), base = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            ty = max(th + base, y1)
            cv2.rectangle(out, (x1, ty - th - base), (x1 + tw + 4, ty), c, -1)
            cv2.putText(out, label, (x1 + 2, ty - base), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 0, 0), 1, cv2.LINE_AA)
        return out
