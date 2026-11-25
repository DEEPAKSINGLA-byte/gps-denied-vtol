#!/usr/bin/env python3
"""
segmentation_node.py - YOLO + SAM instance segmentation on the ground laptop (GPU).

Subscribes to the drone camera stream that camera_tcp_receiver publishes, so it never
opens its own socket (live_segmentation.py's TCP server on port 5000 would compete with
camera_tcp_receiver for the Pi's single camera connection).

    /camera/image_raw ──> [YOLOv8-seg] ──boxes──> [SAM 2.1] ──> fuse ──> /segmentation/*

YOLO supplies classes, boxes and coarse masks; SAM is prompted with YOLO's boxes and
returns sharp boundaries. Each SAM mask is kept only if it passes quality gates,
otherwise the YOLO mask is used (see hybrid_segmenter.py). sam_model:=none = YOLO only.

Outputs (all stamped with the SOURCE camera frame's header, so they sync with TF/odom)
  /segmentation/detections       vision_msgs/Detection2DArray
                                 hypothesis.class_id = class NAME ("person", "car", ...)
                                 hypothesis.score    = confidence
                                 bbox                = pixel box (center + size)
                                 id                  = index into /segmentation/instance_mask
  /segmentation/class_mask       sensor_msgs/Image mono8, pixel = class_id + 1 (0 = background)
  /segmentation/instance_mask    sensor_msgs/Image mono8, pixel = detection index + 1
  /segmentation/image_annotated  sensor_msgs/Image bgr8 overlay (only computed if subscribed)
  /segmentation/class_names      std_msgs/String JSON {"0": "person", ...} (latched)
  /metrics/segmentation/{inference_ms, latency_ms, rate_hz, num_detections}
                                 inference_ms = total YOLO + SAM + fusion time
  /metrics/segmentation/{yolo_ms, sam_ms, sam_refined}
                                 per-stage time; sam_refined = masks taken from SAM

Only the newest frame is processed: if inference is slower than the camera, older
frames are dropped instead of queueing (latency stays bounded).
"""

import json
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String
from vision_msgs.msg import (Detection2D, Detection2DArray,
                             ObjectHypothesisWithPose)

from quadcopter_perception.hybrid_segmenter import HybridSegmenter


class SegmentationNode(Node):

    def __init__(self):
        super().__init__('segmentation_node')
        P = self.declare_parameter
        P('image_topic', '/camera/image_raw')
        P('model', 'yolov8n-seg.pt')     # path, or a name Ultralytics downloads
        P('device', '')                  # '' = cuda:0 if available else cpu
        P('conf', 0.35)
        P('iou', 0.45)
        P('imgsz', 640)
        P('half', True)                  # FP16 on CUDA
        P('classes', [''])               # names to keep, e.g. ['person']; [''] = all
        # --- SAM refinement (see hybrid_segmenter.py for the fusion gates) ---
        P('sam_model', 'sam2.1_t.pt')    # 'none' / '' = disable SAM (plain YOLO masks)
        P('sam_imgsz', 1024)             # 768 ~2x faster, slightly softer boundaries
        P('sam_max_boxes', 8)            # refine at most N most-confident detections
        P('sam_min_box_px', 16)          # smaller boxes keep the YOLO mask
        P('sam_min_score', 0.5)          # SAM predicted-IoU threshold
        P('sam_min_agreement', 0.3)      # min IoU(SAM, YOLO) to accept the SAM mask
        P('sam_box_margin', 0.1)         # SAM mask clipped to box grown by this fraction
        P('sam_every_n', 1)              # run SAM on every Nth processed frame
        g = lambda n: self.get_parameter(n).value

        # Heavy imports (torch/ultralytics) + warm-up happen inside HybridSegmenter
        self.seg = HybridSegmenter(
            yolo_model=str(g('model')), sam_model=str(g('sam_model')),
            device=str(g('device')), conf=float(g('conf')), iou=float(g('iou')),
            imgsz=int(g('imgsz')), half=bool(g('half')),
            classes=[c for c in g('classes') if c],
            sam_imgsz=int(g('sam_imgsz')), sam_max_boxes=int(g('sam_max_boxes')),
            sam_min_box_px=int(g('sam_min_box_px')),
            sam_min_score=float(g('sam_min_score')),
            sam_min_agreement=float(g('sam_min_agreement')),
            sam_box_margin=float(g('sam_box_margin')),
            logger=self.get_logger().info)
        self.names = self.seg.names
        self.class_ids = self.seg.class_ids
        self.sam_every_n = max(1, int(g('sam_every_n')))
        self._frame_idx = 0

        self.bridge = CvBridge()
        # RELIABLE on purpose: with CycloneDDS and default kernel socket buffers
        # (net.core.rmem_max = 208 KB) a BEST_EFFORT 640x480 image is fragmented and
        # dropped - measured 0 frames delivered across processes. depth=1 still keeps
        # only the newest frame.
        cam_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Image, str(g('image_topic')), self._on_image, cam_qos)

        self.det_pub = self.create_publisher(Detection2DArray, '/segmentation/detections', 10)
        self.cls_pub = self.create_publisher(Image, '/segmentation/class_mask', 2)
        self.inst_pub = self.create_publisher(Image, '/segmentation/instance_mask', 2)
        self.ann_pub = self.create_publisher(Image, '/segmentation/image_annotated', 2)
        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        names_pub = self.create_publisher(String, '/segmentation/class_names', latched)
        names_pub.publish(String(data=json.dumps({str(k): v for k, v in self.names.items()})))
        self.m = {k: self.create_publisher(Float32, f'/metrics/segmentation/{k}', 10)
                  for k in ('inference_ms', 'latency_ms', 'rate_hz', 'num_detections',
                            'yolo_ms', 'sam_ms', 'sam_refined')}

        # latest-frame hand-off to a worker thread (drop stale frames)
        self._lock = threading.Lock()
        self._latest = None
        self._event = threading.Event()
        self._running = True
        self._stamps = []
        self._worker = threading.Thread(target=self._loop, daemon=True)
        self._worker.start()

        kept = 'all' if self.class_ids is None else [self.names[i] for i in self.class_ids]
        sam = (f"{g('sam_model')} @ {g('sam_imgsz')}px, every {self.sam_every_n} frame(s)"
               if self.seg.sam_enabled else 'disabled')
        self.get_logger().info(f"Segmentation ready: {g('image_topic')} -> /segmentation/* "
                               f'(classes: {kept}; SAM: {sam})')

    # ------------------------------------------------------------------ input
    def _on_image(self, msg: Image):
        with self._lock:
            self._latest = msg
        self._event.set()

    def _loop(self):
        while self._running and rclpy.ok():
            if not self._event.wait(0.5):
                continue
            self._event.clear()
            with self._lock:
                msg, self._latest = self._latest, None
            if msg is None:
                continue
            try:
                self._process(msg)
            except Exception as e:      # keep the node alive on a bad frame
                self.get_logger().warn(f'Segmentation failed on a frame: {e}',
                                       throttle_duration_sec=5.0)

    # -------------------------------------------------------------- inference
    def _process(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        refine = self._frame_idx % self.sam_every_n == 0
        self._frame_idx += 1
        res = self.seg.segment(frame, refine=refine)
        n = len(res.instances)

        det_arr = Detection2DArray()
        det_arr.header = msg.header
        for i, inst in enumerate(res.instances):
            d = Detection2D()
            d.header = msg.header
            d.id = str(i)
            x1, y1, x2, y2 = (float(v) for v in inst.box)
            d.bbox.center.position.x = (x1 + x2) / 2.0
            d.bbox.center.position.y = (y1 + y2) / 2.0
            d.bbox.size_x = x2 - x1
            d.bbox.size_y = y2 - y1
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = inst.class_name
            hyp.hypothesis.score = inst.score
            d.results.append(hyp)
            det_arr.detections.append(d)

        self.det_pub.publish(det_arr)
        for pub, img in ((self.cls_pub, res.class_mask), (self.inst_pub, res.instance_mask)):
            out = self.bridge.cv2_to_imgmsg(img, encoding='mono8')
            out.header = msg.header
            pub.publish(out)
        if self.ann_pub.get_subscription_count() > 0:
            out = self.bridge.cv2_to_imgmsg(self.seg.annotate(frame, res), encoding='bgr8')
            out.header = msg.header
            self.ann_pub.publish(out)

        # metrics
        now = time.monotonic()
        self._stamps = [t for t in self._stamps if now - t < 2.0] + [now]
        rate = (len(self._stamps) - 1) / (now - self._stamps[0]) if len(self._stamps) > 1 else 0.0
        age_ms = (self.get_clock().now() - Time.from_msg(msg.header.stamp)).nanoseconds / 1e6
        t = res.timing_ms
        for k, v in (('inference_ms', t['total']), ('latency_ms', age_ms),
                     ('rate_hz', rate), ('num_detections', n),
                     ('yolo_ms', t['yolo']), ('sam_ms', t['sam']),
                     ('sam_refined', res.num_sam_refined)):
            self.m[k].publish(Float32(data=float(v)))

    def destroy_node(self):
        self._running = False
        self._event.set()
        self._worker.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SegmentationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
