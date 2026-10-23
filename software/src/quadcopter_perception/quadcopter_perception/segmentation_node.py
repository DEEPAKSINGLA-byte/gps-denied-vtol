"""ROS 2 image segmentation node backed by Ultralytics YOLO segmentation."""

import json
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO


class SegmentationNode(Node):
    """Subscribe to camera images and publish segmentation visualizations."""

    def __init__(self):
        super().__init__('quadcopter_segmentation')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('model', 'yolov8n-seg.pt')
        self.declare_parameter('device', '')
        self.declare_parameter('confidence', 0.35)
        self.declare_parameter('max_rate_hz', 5.0)

        self.image_topic = self.get_parameter('image_topic').value
        model_path = self.get_parameter('model').value
        self.device = self.get_parameter('device').value
        self.confidence = float(self.get_parameter('confidence').value)
        self.max_rate_hz = max(0.1, float(self.get_parameter('max_rate_hz').value))

        self.bridge = CvBridge()
        self.model = YOLO(model_path)
        self.last_inference_ns = 0
        self.annotated_pub = self.create_publisher(Image, '~/annotated', 10)
        self.mask_pub = self.create_publisher(Image, '~/class_mask', 10)
        self.detections_pub = self.create_publisher(String, '~/detections', 10)
        self.image_sub = self.create_subscription(Image, self.image_topic, self.image_callback, 5)
        self.get_logger().info(
            f'YOLO segmentation ready: {self.image_topic} -> '
            f'{self.get_name()}/annotated, class_mask, detections'
        )

    def image_callback(self, msg):
        """Infer on a camera frame and publish images plus structured detections."""
        now_ns = time.monotonic_ns()
        min_period_ns = int(1e9 / self.max_rate_hz)
        if now_ns - self.last_inference_ns < min_period_ns:
            return
        self.last_inference_ns = now_ns

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            kwargs = {'conf': self.confidence, 'verbose': False}
            if self.device:
                kwargs['device'] = self.device
            result = self.model(frame, **kwargs)[0]

            annotated = result.plot()
            class_mask = self._make_class_mask(result, frame.shape[:2])
            detections = self._make_detection_list(result)

            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            annotated_msg.header = msg.header
            mask_msg = self.bridge.cv2_to_imgmsg(class_mask, encoding='rgb8')
            mask_msg.header = msg.header
            self.annotated_pub.publish(annotated_msg)
            self.mask_pub.publish(mask_msg)
            output = String()
            output.data = json.dumps(detections)
            self.detections_pub.publish(output)
        except Exception as exc:  # Keep one malformed frame from killing the node.
            self.get_logger().error(f'Could not segment camera frame: {exc}')

    @staticmethod
    def _make_class_mask(result, image_shape):
        """Return a black background with a deterministic color per class mask."""
        height, width = image_shape
        output = np.zeros((height, width, 3), dtype=np.uint8)
        if result.masks is None or result.boxes is None:
            return output

        classes = result.boxes.cls.detach().cpu().numpy().astype(int)
        masks = result.masks.data.detach().cpu().numpy()
        for mask, class_id in zip(masks, classes):
            resized = cv2.resize(mask, (width, height), interpolation=cv2.INTER_NEAREST)
            hue = (int(class_id) * 47 + 20) % 180
            color = cv2.cvtColor(np.uint8([[[hue, 220, 255]]]), cv2.COLOR_HSV2RGB)[0, 0]
            output[resized > 0.5] = color
        return output

    @staticmethod
    def _make_detection_list(result):
        """Convert YOLO boxes into a JSON-serializable list."""
        detections = []
        if result.boxes is None:
            return detections
        boxes = result.boxes
        xyxy = boxes.xyxy.detach().cpu().numpy()
        classes = boxes.cls.detach().cpu().numpy().astype(int)
        scores = boxes.conf.detach().cpu().numpy()
        for box, class_id, score in zip(xyxy, classes, scores):
            detections.append({
                'class_id': int(class_id),
                'class_name': result.names[int(class_id)],
                'confidence': float(score),
                'bbox_xyxy': [float(value) for value in box],
            })
        return detections


def main(args=None):
    """Run the image segmentation node."""
    rclpy.init(args=args)
    node = SegmentationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
