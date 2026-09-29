#!/usr/bin/env python3
"""Node A: Face detection on Azure Kinect RGB image → gaze topic.

Subscribes to the camera image, runs OpenCV Haar cascade face detection,
selects a target face, and publishes normalized gaze coordinates.
"""

import math
import os
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from spot_eye_animation_msgs.msg import Gaze

_CASCADE_NAME = 'haarcascade_frontalface_default.xml'


def _clamp(val, lo, hi):
    return max(lo, min(hi, val))


def _resolve_cascade(param_value: str, logger) -> str:
    """Find the Haar cascade XML by checking candidates in priority order.

    Returns the first path that exists on disk, or raises FileNotFoundError.
    """
    candidates = []

    if param_value:
        candidates.append(param_value)

    # cv2.data (pip-installed OpenCV provides this)
    cv2_data = getattr(cv2, 'data', None)
    haar_dir = getattr(cv2_data, 'haarcascades', None) if cv2_data else None
    if haar_dir:
        candidates.append(str(Path(haar_dir) / _CASCADE_NAME))

    # Common system locations
    candidates += [
        f'/usr/share/opencv4/haarcascades/{_CASCADE_NAME}',
        f'/usr/share/opencv/haarcascades/{_CASCADE_NAME}',
        f'/usr/local/share/opencv4/haarcascades/{_CASCADE_NAME}',
    ]

    for p in candidates:
        if p and Path(p).is_file():
            return p

    raise FileNotFoundError(
        f'Haar cascade not found. Tried: {candidates}'
    )


class FaceGazeNode(Node):
    def __init__(self):
        super().__init__('face_gaze')

        # ── Parameters ───────────────────────────────────────────────────
        self.declare_parameter('image_topic', 'rgb/image_raw')
        self.declare_parameter('gaze_topic', 'eye_animation/gaze')
        self.declare_parameter('detection_hz', 10.0)
        self.declare_parameter('cascade_path', '')
        self.declare_parameter('min_face_size', 60)
        self.declare_parameter('scale_factor', 1.2)
        self.declare_parameter('min_neighbors', 5)
        self.declare_parameter('invert_x', False)
        self.declare_parameter('invert_y', False)
        self.declare_parameter('gaze_scale_x', 1.0)
        self.declare_parameter('gaze_scale_y', 1.0)
        self.declare_parameter('gaze_offset_x', 0.0)
        self.declare_parameter('gaze_offset_y', 0.0)

        # ── OpenCV diagnostics ──────────────────────────────────────────
        self.get_logger().info(
            f'cv2: version={cv2.__version__}, '
            f'file={cv2.__file__}, '
            f'has cv2.data={hasattr(cv2, "data")}'
        )

        # ── Load cascade ─────────────────────────────────────────────────
        self._detection_enabled = True
        cascade_param = self.get_parameter('cascade_path').value
        try:
            cascade_path = _resolve_cascade(cascade_param, self.get_logger())
        except FileNotFoundError as e:
            self.get_logger().error(str(e))
            self._detection_enabled = False
            self.cascade = None
            self.get_logger().error(
                'Face detection DISABLED — will publish detected=false'
            )
        else:
            self.cascade = cv2.CascadeClassifier(cascade_path)
            if self.cascade.empty():
                self.get_logger().error(
                    f'CascadeClassifier loaded but empty: {cascade_path}'
                )
                self._detection_enabled = False
                self.cascade = None
                self.get_logger().error(
                    'Face detection DISABLED — will publish detected=false'
                )
            else:
                self.get_logger().info(f'Loaded cascade: {cascade_path}')

        # ── Subscription (latest-frame buffer) ───────────────────────────
        self.bridge = CvBridge()
        self._latest_msg = None  # Atomic swap; safe in SingleThreadedExecutor
        self._encoding_warned = False

        image_topic = self.get_parameter('image_topic').value
        self.create_subscription(
            Image, image_topic, self._image_cb, qos_profile_sensor_data
        )
        self.get_logger().info(f'Subscribing to: {image_topic}')

        # ── Publisher ────────────────────────────────────────────────────
        gaze_topic = self.get_parameter('gaze_topic').value
        self.gaze_pub = self.create_publisher(Gaze, gaze_topic, 10)

        # ── Detection timer ──────────────────────────────────────────────
        hz = self.get_parameter('detection_hz').value
        self.create_timer(1.0 / hz, self._detect)

        # ── Tracking state ───────────────────────────────────────────────
        self._prev_center = None   # (x, y) in downscaled coords
        self._prev_size = 0        # width of previous target face
        self._prev_lost_time = 0.0
        self._last_gaze_x = 0.0
        self._last_gaze_y = 0.0
        self._hysteresis = 0.5     # seconds before switching target
        self._last_detected_time = 0.0
        self._holdover = 1.0       # keep publishing detected=true for this many seconds after last real detection

    def _image_cb(self, msg: Image):
        """Store latest image. Does NOT run detection."""
        self._latest_msg = msg

    def _detect(self):
        """Run face detection on the latest frame and publish gaze."""
        if not self._detection_enabled:
            self._publish_gaze(0.0, 0.0, False, 0)
            return

        msg = self._latest_msg
        if msg is None:
            # No image yet — still publish detected=false
            self._publish_gaze(self._last_gaze_x, self._last_gaze_y, False, 0)
            return
        self._latest_msg = None  # consume

        # ── Convert image ────────────────────────────────────────────────
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            if not self._encoding_warned:
                self.get_logger().warn(
                    f'cv_bridge conversion issue (encoding={msg.encoding}): {e}'
                )
                self._encoding_warned = True
            self._publish_gaze(self._last_gaze_x, self._last_gaze_y, False, 0)
            return

        # ── Downscale ────────────────────────────────────────────────────
        target_w = 320
        h, w = frame.shape[:2]
        if w <= 0:
            return
        scale = target_w / w
        small = cv2.resize(frame, (target_w, int(h * scale)), interpolation=cv2.INTER_AREA)
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)

        # ── Detect faces ─────────────────────────────────────────────────
        min_size = self.get_parameter('min_face_size').value
        scale_factor = self.get_parameter('scale_factor').value
        min_neighbors = self.get_parameter('min_neighbors').value

        faces = self.cascade.detectMultiScale(
            gray,
            scaleFactor=scale_factor,
            minNeighbors=min_neighbors,
            minSize=(min_size, min_size),
        )

        num_faces = len(faces) if isinstance(faces, np.ndarray) else 0

        if num_faces == 0:
            self._prev_lost_time = time.monotonic()
            # Holdover: keep reporting detected=true briefly so renderer
            # doesn't flicker to idle on every missed frame
            holdover_active = (time.monotonic() - self._last_detected_time) < self._holdover
            self._publish_gaze(self._last_gaze_x, self._last_gaze_y, holdover_active, 0)
            return

        # ── Select target face ───────────────────────────────────────────
        target = self._select_face(faces)
        fx, fy, fw, fh = target
        face_cx = fx + fw / 2.0
        face_cy = fy + fh / 2.0

        # Update tracking state
        self._prev_center = (face_cx, face_cy)
        self._prev_size = fw
        self._last_detected_time = time.monotonic()

        # ── Normalize to [-1, 1] using downscaled dimensions ─────────────
        sh, sw = small.shape[:2]
        gaze_x = (face_cx / sw - 0.5) * 2.0
        gaze_y = (face_cy / sh - 0.5) * 2.0

        # ── Apply calibration ────────────────────────────────────────────
        if self.get_parameter('invert_x').value:
            gaze_x = -gaze_x
        if self.get_parameter('invert_y').value:
            gaze_y = -gaze_y

        gaze_x = _clamp(
            gaze_x * self.get_parameter('gaze_scale_x').value
            + self.get_parameter('gaze_offset_x').value,
            -1.0, 1.0,
        )
        gaze_y = _clamp(
            gaze_y * self.get_parameter('gaze_scale_y').value
            + self.get_parameter('gaze_offset_y').value,
            -1.0, 1.0,
        )

        self._last_gaze_x = gaze_x
        self._last_gaze_y = gaze_y
        self._publish_gaze(gaze_x, gaze_y, True, num_faces)

    def _select_face(self, faces):
        """Select which face to track. Nearest-center to previous target,
        with hysteresis before switching. Falls back to largest face."""
        now = time.monotonic()

        if self._prev_center is not None:
            # Find face closest to previous target center
            best_idx = None
            best_dist = float('inf')
            for i, (x, y, w, h) in enumerate(faces):
                cx = x + w / 2.0
                cy = y + h / 2.0
                dx = cx - self._prev_center[0]
                dy = cy - self._prev_center[1]
                dist = math.sqrt(dx * dx + dy * dy)
                if dist < best_dist:
                    best_dist = dist
                    best_idx = i

            # Accept if within 1.5x previous face size
            threshold = self._prev_size * 1.5 if self._prev_size > 0 else float('inf')
            if best_dist < threshold:
                return faces[best_idx]

        # Fallback: largest face (closest to camera)
        areas = [w * h for (x, y, w, h) in faces]
        return faces[np.argmax(areas)]

    def _publish_gaze(self, x: float, y: float, detected: bool, num_faces: int):
        """Publish a Gaze message."""
        msg = Gaze()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.x = float(x)
        msg.y = float(y)
        msg.detected = detected
        msg.num_faces = num_faces
        self.gaze_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FaceGazeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
