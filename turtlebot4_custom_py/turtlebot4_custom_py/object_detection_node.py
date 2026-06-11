#!/usr/bin/env python3

"""Object detection node using YOLOv8 and the Oak-D stereo camera.

Subscribes to synchronized RGB + depth images, runs YOLO inference,
projects detections into the map frame via TF, and publishes:
  - /detected_objects_markers (MarkerArray) for RViz visualization
  - /detected_objects (String, JSON) for downstream nodes
"""

import json

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped

from cv_bridge import CvBridge
import message_filters
import tf2_ros
import tf2_geometry_msgs  # noqa: F401  — registers PointStamped transform

from ultralytics import YOLO


class ObjectDetectionNode(Node):

    def __init__(self):
        super().__init__('object_detection_node')

        # -- Parameters --
        self.declare_parameter('model', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('image_topic', 'oakd/rgb/preview/image_raw')
        self.declare_parameter('depth_topic', 'oakd/rgb/preview/depth')
        self.declare_parameter('camera_info_topic', 'oakd/rgb/preview/camera_info')
        self.declare_parameter('camera_frame', 'oakd_rgb_camera_optical_frame')
        self.declare_parameter('detection_interval', 0.2)

        model_path = self.get_parameter('model').value
        self.conf_thresh = self.get_parameter('confidence_threshold').value
        image_topic = self.get_parameter('image_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        self.camera_frame = self.get_parameter('camera_frame').value
        det_interval = self.get_parameter('detection_interval').value

        # -- YOLO model --
        self.get_logger().info(f'Loading YOLO model: {model_path}')
        self.model = YOLO(model_path)
        self.get_logger().info('YOLO model loaded')

        self.bridge = CvBridge()

        # Camera intrinsics (populated by CameraInfo callback)
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # -- TF --
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # -- Subscribers --
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info_callback,
            10,
        )

        # Synchronized RGB + Depth
        self.image_sub = message_filters.Subscriber(
            self, Image, image_topic, qos_profile=qos_profile_sensor_data)
        self.depth_sub = message_filters.Subscriber(
            self, Image, depth_topic, qos_profile=qos_profile_sensor_data)

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.depth_sub],
            queue_size=10,
            slop=0.1,
        )
        self.sync.registerCallback(self.synced_callback)

        # -- Publishers --
        self.marker_pub = self.create_publisher(
            MarkerArray, 'detected_objects_markers', 10)
        self.detections_pub = self.create_publisher(
            String, 'detected_objects', 10)

        # -- Throttle --
        self.last_detection_time = self.get_clock().now()
        self.detection_interval = Duration(seconds=det_interval)

        self.get_logger().info('Object Detection Node is ready!')

    # ------------------------------------------------------------------ #
    #  Callbacks
    # ------------------------------------------------------------------ #

    def camera_info_callback(self, msg: CameraInfo):
        """Store camera intrinsics from the K matrix."""
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def synced_callback(self, image_msg: Image, depth_msg: Image):
        """Process a synchronized RGB + depth pair."""
        now = self.get_clock().now()
        if (now - self.last_detection_time) < self.detection_interval:
            return
        self.last_detection_time = now

        # Convert ROS images to OpenCV
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        # Handle uint16 depth in millimeters (real Oak-D hardware)
        if depth_image.dtype == np.uint16:
            depth_image = depth_image.astype(np.float32) / 1000.0

        # Run YOLO
        results = self.model(cv_image, conf=self.conf_thresh, verbose=False)

        if len(results) == 0 or len(results[0].boxes) == 0:
            return

        detections = []
        markers = MarkerArray()
        marker_id = 0

        for box in results[0].boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            cx_px = int((x1 + x2) / 2)
            cy_px = int((y1 + y2) / 2)
            conf = float(box.conf[0])
            class_id = int(box.cls[0])
            label = self.model.names[class_id]

            # Sample depth at the bounding box center
            if not (0 <= cy_px < depth_image.shape[0] and
                    0 <= cx_px < depth_image.shape[1]):
                continue

            depth = float(depth_image[cy_px, cx_px])
            if depth <= 0.0 or np.isnan(depth) or np.isinf(depth) or depth > 10.0:
                continue

            detection = {
                'label': label,
                'confidence': round(conf, 2),
                'depth_m': round(depth, 2),
            }

            # Project into map frame if we have camera intrinsics
            if self.fx is not None:
                marker = self._project_to_map(
                    cx_px, cy_px, depth, label, conf,
                    image_msg.header, now, marker_id,
                    detection,
                )
                if marker is not None:
                    markers.markers.append(marker)
                    marker_id += 1

            detections.append(detection)

        # Publish
        if markers.markers:
            self.marker_pub.publish(markers)

        if detections:
            msg = String()
            msg.data = json.dumps(detections)
            self.detections_pub.publish(msg)
            self.get_logger().info(
                f'Detected: {", ".join(d["label"] for d in detections)}')

    # ------------------------------------------------------------------ #
    #  Helpers
    # ------------------------------------------------------------------ #

    def _project_to_map(self, cx_px, cy_px, depth, label, conf,
                        header, stamp, marker_id, detection):
        """Deproject a pixel + depth into the map frame.

        Returns a Marker for RViz or None if the TF lookup fails.
        Also updates the detection dict with map coordinates.
        """
        z = depth
        x = (cx_px - self.cx) * z / self.fx
        y = (cy_px - self.cy) * z / self.fy

        point_cam = PointStamped()
        point_cam.header = header
        point_cam.header.frame_id = self.camera_frame
        point_cam.point.x = float(x)
        point_cam.point.y = float(y)
        point_cam.point.z = float(z)

        try:
            point_map = self.tf_buffer.transform(
                point_cam, 'map', timeout=Duration(seconds=0.1))
        except Exception as e:
            self.get_logger().debug(f'TF lookup failed: {e}')
            return None

        detection['map_x'] = round(point_map.point.x, 2)
        detection['map_y'] = round(point_map.point.y, 2)
        detection['map_z'] = round(point_map.point.z, 2)

        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = stamp.to_msg()
        marker.ns = 'detections'
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position = point_map.point
        marker.pose.position.z += 0.3
        marker.scale.z = 0.2
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.text = f'{label} ({conf:.0%})'
        marker.lifetime = Duration(seconds=2.0).to_msg()

        return marker


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ObjectDetectionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
