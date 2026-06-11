#!/usr/bin/env python3
"""Run a YOLO model on a camera stream and publish what it sees.

Inference runs on the LAPTOP (ultralytics, CPU-only torch, ~17 FPS for
yolo26n), not on the OAK-D's VPU — on-device inference would mean converting
the model to a Myriad blob and changing the camera pipeline on the robot's
Pi. Subscribing to the image topic instead works with the OAK-D, the laptop
webcam, and recorded bags alike.

Sources (pick one):
  - ROS topic (default): the OAK-D Pro stream, `image_topic` parameter
  - laptop webcam: pass `webcam:=0` (the /dev/videoN index)

Publishes:
  - `detections` (vision_msgs/Detection2DArray): bbox center/size in pixels,
    class name + confidence per detection — what a follow controller consumes
  - `detections/image` (sensor_msgs/Image): annotated frames, viewable in RViz
    (only computed when someone is subscribed)

Run from the workspace root so the default model path resolves:
  ros2 run turtlebot4_custom_py yolo_detection
  ros2 run turtlebot4_custom_py yolo_detection --ros-args -p webcam:=0 -p classes:=person
"""
from pathlib import Path

import cv2
import rclpy

from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from ultralytics import YOLO
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


class YoloDetectionNode(Node):

    def __init__(self):
        super().__init__('yolo_detection')
        self.declare_parameter('image_topic', '/oakd/rgb/preview/image_raw')
        self.declare_parameter('webcam', -1)
        self.declare_parameter('model', 'models/yolo26n.pt')
        self.declare_parameter('conf', 0.4)
        self.declare_parameter('rate', 10.0)
        # Comma-separated class names to keep, e.g. 'person' or 'person,dog'.
        # Empty keeps every class the model knows.
        self.declare_parameter('classes', '')

        model_path = self.get_parameter('model').value
        if not Path(model_path).exists():
            raise FileNotFoundError(
                f'YOLO model not found at {model_path} — run from the '
                'workspace root (like the launches), or pass -p model:=<path>')
        self.model = YOLO(model_path)

        wanted = [c.strip() for c in
                  self.get_parameter('classes').value.split(',') if c.strip()]
        name_to_id = {name: i for i, name in self.model.names.items()}
        unknown = [c for c in wanted if c not in name_to_id]
        if unknown:
            raise ValueError(
                f'Model has no class(es) {unknown}; it knows: '
                f'{sorted(name_to_id)}')
        self.class_filter = [name_to_id[c] for c in wanted] or None
        self.conf = float(self.get_parameter('conf').value)

        self.bridge = CvBridge()
        self.latest = None  # (header, bgr frame) not yet run through the model

        webcam = int(self.get_parameter('webcam').value)
        if webcam >= 0:
            self.cap = cv2.VideoCapture(webcam)
            if not self.cap.isOpened():
                raise RuntimeError(f'Could not open webcam {webcam}')
            self.get_logger().info(f'Source: webcam {webcam}')
        else:
            self.cap = None
            topic = self.get_parameter('image_topic').value
            self.create_subscription(
                Image, topic, self.image_callback, qos_profile_sensor_data)
            self.get_logger().info(f'Source: image topic {topic}')

        self.detections_pub = self.create_publisher(
            Detection2DArray, 'detections', 10)
        self.annotated_pub = self.create_publisher(
            Image, 'detections/image', qos_profile_sensor_data)
        self.create_timer(1.0 / float(self.get_parameter('rate').value),
                          self.detect)

    def image_callback(self, msg):
        # Only stash the frame; inference happens in the timer at `rate`, so
        # a fast camera can't queue up stale frames behind a slow CPU.
        self.latest = (msg.header, self.bridge.imgmsg_to_cv2(msg, 'bgr8'))

    def detect(self):
        if self.cap is not None:
            ok, frame = self.cap.read()
            if not ok:
                self.get_logger().warning('Webcam read failed',
                                          throttle_duration_sec=5.0)
                return
            header = None
        elif self.latest is not None:
            header, frame = self.latest
            self.latest = None
        else:
            self.get_logger().info('Waiting for images...',
                                   throttle_duration_sec=5.0)
            return

        result = self.model.predict(frame, conf=self.conf,
                                    classes=self.class_filter,
                                    verbose=False)[0]

        msg = Detection2DArray()
        if header is not None:
            msg.header = header
        else:
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'webcam'

        counts = {}
        for box in result.boxes:
            name = self.model.names[int(box.cls)]
            counts[name] = counts.get(name, 0) + 1

            det = Detection2D(header=msg.header)
            x, y, w, h = (float(v) for v in box.xywh[0])
            det.bbox.center.position.x = x
            det.bbox.center.position.y = y
            det.bbox.size_x = w
            det.bbox.size_y = h
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = name
            hyp.hypothesis.score = float(box.conf)
            det.results.append(hyp)
            msg.detections.append(det)
        self.detections_pub.publish(msg)

        if counts:
            summary = ', '.join(f'{n}x {name}' for name, n in counts.items())
            self.get_logger().info(f'Detected: {summary}',
                                   throttle_duration_sec=2.0)

        # result.plot() is not free, so only annotate for actual viewers
        if self.annotated_pub.get_subscription_count() > 0:
            annotated = self.bridge.cv2_to_imgmsg(result.plot(),
                                                  encoding='bgr8')
            annotated.header = msg.header
            self.annotated_pub.publish(annotated)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # Ctrl-C makes rclpy shut the context down itself; plain shutdown()
        # would raise "rcl_shutdown already called"
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
