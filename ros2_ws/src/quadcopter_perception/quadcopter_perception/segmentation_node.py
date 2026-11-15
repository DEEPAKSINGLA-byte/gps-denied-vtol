#!/usr/bin/env python3
"""
segmentation_node.py - YOLOv8 instance segmentation on the ground laptop (GPU).

Subscribes to the drone camera stream that camera_tcp_receiver publishes, so it never
opens its own socket (live_segmentation.py's TCP server on port 5000 would compete with
camera_tcp_receiver for the Pi's single camera connection).

    /camera/image_raw ──> [YOLOv8-seg] ──> /segmentation/*

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

Only the newest frame is processed: if inference is slower than the camera, older
frames are dropped instead of queueing (latency stays bounded).
"""

import json
import threading
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String
from vision_msgs.msg import (Detection2D, Detection2DArray,
                             ObjectHypothesisWithPose)


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
        g = lambda n: self.get_parameter(n).value

        import torch                     # heavy imports after param parsing
        from ultralytics import YOLO

        dev = str(g('device')) or ('cuda:0' if torch.cuda.is_available() else 'cpu')
        self.device = dev
        self.half = bool(g('half')) and dev.startswith('cuda')
        self.conf, self.iou, self.imgsz = float(g('conf')), float(g('iou')), int(g('imgsz'))

        self.get_logger().info(f"Loading {g('model')} on {dev} (half={self.half})...")
        self.model = YOLO(str(g('model')))
        self.names = {int(k): v for k, v in self.model.names.items()}

        wanted = [c for c in g('classes') if c]
        unknown = [c for c in wanted if c not in self.names.values()]
        if unknown:
            self.get_logger().warn(f'Unknown class names ignored: {unknown}')
        ids = [i for i, n in self.names.items() if n in wanted]
        self.class_ids = ids or None     # None = all classes

        # Warm-up so the first real frame doesn't pay CUDA/cuDNN init (~1-2 s)
        self.model.predict(np.zeros((480, 640, 3), np.uint8), device=dev, half=self.half,
                           imgsz=self.imgsz, verbose=False)

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
                  for k in ('inference_ms', 'latency_ms', 'rate_hz', 'num_detections')}

        # latest-frame hand-off to a worker thread (drop stale frames)
        self._lock = threading.Lock()
        self._latest = None
        self._event = threading.Event()
        self._running = True
        self._stamps = []
        self._worker = threading.Thread(target=self._loop, daemon=True)
        self._worker.start()

        kept = 'all' if self.class_ids is None else [self.names[i] for i in self.class_ids]
        self.get_logger().info(f"Segmentation ready: {g('image_topic')} -> /segmentation/* "
                               f'(classes: {kept})')

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
        h, w = frame.shape[:2]

        t0 = time.perf_counter()
        res = self.model.predict(frame, device=self.device, half=self.half, conf=self.conf,
                                 iou=self.iou, imgsz=self.imgsz, classes=self.class_ids,
                                 retina_masks=True, verbose=False)[0]
        infer_ms = (time.perf_counter() - t0) * 1e3

        boxes = res.boxes
        n = 0 if boxes is None else len(boxes)
        cls = boxes.cls.int().cpu().numpy() if n else np.zeros(0, int)
        conf = boxes.conf.cpu().numpy() if n else np.zeros(0)
        xyxy = boxes.xyxy.cpu().numpy() if n else np.zeros((0, 4))

        # Masks: paint low -> high confidence so the most confident instance wins overlaps
        class_mask = np.zeros((h, w), np.uint8)
        inst_mask = np.zeros((h, w), np.uint8)
        if n and res.masks is not None:
            masks = res.masks.data.cpu().numpy() > 0.5          # (n, h, w) via retina_masks
            if masks.shape[1:] != (h, w):                       # defensive: resize if needed
                import cv2
                masks = np.stack([cv2.resize(m.astype(np.uint8), (w, h),
                                             interpolation=cv2.INTER_NEAREST) > 0
                                  for m in masks])
            for i in np.argsort(conf):
                class_mask[masks[i]] = cls[i] + 1
                inst_mask[masks[i]] = min(i + 1, 255)

        det_arr = Detection2DArray()
        det_arr.header = msg.header
        for i in range(n):
            d = Detection2D()
            d.header = msg.header
            d.id = str(i)
            x1, y1, x2, y2 = (float(v) for v in xyxy[i])
            d.bbox.center.position.x = (x1 + x2) / 2.0
            d.bbox.center.position.y = (y1 + y2) / 2.0
            d.bbox.size_x = x2 - x1
            d.bbox.size_y = y2 - y1
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = self.names.get(int(cls[i]), str(int(cls[i])))
            hyp.hypothesis.score = float(conf[i])
            d.results.append(hyp)
            det_arr.detections.append(d)

        self.det_pub.publish(det_arr)
        for pub, img in ((self.cls_pub, class_mask), (self.inst_pub, inst_mask)):
            out = self.bridge.cv2_to_imgmsg(img, encoding='mono8')
            out.header = msg.header
            pub.publish(out)
        if self.ann_pub.get_subscription_count() > 0:
            out = self.bridge.cv2_to_imgmsg(res.plot(), encoding='bgr8')
            out.header = msg.header
            self.ann_pub.publish(out)

        # metrics
        now = time.monotonic()
        self._stamps = [t for t in self._stamps if now - t < 2.0] + [now]
        rate = (len(self._stamps) - 1) / (now - self._stamps[0]) if len(self._stamps) > 1 else 0.0
        age_ms = (self.get_clock().now() - Time.from_msg(msg.header.stamp)).nanoseconds / 1e6
        for k, v in (('inference_ms', infer_ms), ('latency_ms', age_ms),
                     ('rate_hz', rate), ('num_detections', n)):
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
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
