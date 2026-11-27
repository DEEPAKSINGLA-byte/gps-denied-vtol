#!/usr/bin/env python3
"""Timing benchmark of the project's HybridSegmenter (YOLO + SAM 2.1) on this GPU.

Runs each configuration on the same 640x480 frame, discards warm-up iterations, and
writes per-configuration statistics to report/data/seg_benchmark.csv.
Usage (from the repository root):
    python3 report/scripts/bench_segmentation.py [--iters 100] [--image path]
"""
import argparse, csv, os, sys, time
import cv2, numpy as np, torch

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, 'ros2_ws/src/quadcopter_perception'))
from quadcopter_perception.hybrid_segmenter import HybridSegmenter  # noqa: E402
import ultralytics  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('--iters', type=int, default=100)
ap.add_argument('--warmup', type=int, default=10)
ap.add_argument('--image', default=os.path.join(os.path.dirname(ultralytics.__file__), 'assets', 'bus.jpg'))
a = ap.parse_args()

frame = cv2.resize(cv2.imread(a.image), (640, 480))
W = lambda n: os.path.join(ROOT, 'ros2_ws', n)
configs = [('YOLO only', ''), ('YOLO+SAM2.1-t @768', 768), ('YOLO+SAM2.1-t @1024', 1024)]
rows = []
for name, imgsz in configs:
    kw = dict(yolo_model=W('yolov8n-seg.pt'), sam_model=(W('sam2.1_t.pt') if imgsz else 'none'))
    if imgsz:
        kw['sam_imgsz'] = imgsz
    seg = HybridSegmenter(**kw)
    for _ in range(a.warmup):
        seg.segment(frame)
    tot, yolo, sam, nref, ninst = [], [], [], [], []
    for _ in range(a.iters):
        torch.cuda.synchronize()
        t0 = time.perf_counter()
        r = seg.segment(frame)
        torch.cuda.synchronize()
        tot.append((time.perf_counter() - t0) * 1e3)
        yolo.append(r.timing_ms.get('yolo', np.nan)); sam.append(r.timing_ms.get('sam', 0.0))
        nref.append(r.num_sam_refined); ninst.append(len(r.instances))
    tot = np.array(tot)
    rows.append(dict(config=name, n=a.iters, mean_ms=tot.mean(), median_ms=np.median(tot),
                     p95_ms=np.percentile(tot, 95), std_ms=tot.std(), fps=1e3 / tot.mean(),
                     yolo_ms=np.nanmean(yolo), sam_ms=np.mean(sam),
                     instances=np.mean(ninst), sam_refined=np.mean(nref)))
    print(rows[-1], flush=True)
    del seg; torch.cuda.empty_cache()

out = os.path.join(ROOT, 'report', 'data', 'seg_benchmark.csv')
with open(out, 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
print('GPU:', torch.cuda.get_device_name(0), '| torch', torch.__version__, '| ultralytics', ultralytics.__version__)
print('wrote', out)
