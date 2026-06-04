#!/usr/bin/env python3
"""
Task 2 – Inspect Damage (Oil Leak Detection)
═════════════════════════════════════════════
Using its cameras or sensors mounted on the arm, the robot identifies the
damaged pipe and confirms the leak visually.

Sequence:
  1. Move arm to INSPECT_READY position (camera facing the pipe)
  2. Subscribe to the camera image topic
  3. Run oil-leak detection via HSV thresholding on incoming frames
  4. Publish detection results on /oil_leak_detected topic
  5. Log the detection status

Engineering Rationale:
  • HSV colour space is used because it separates colour (Hue) from brightness
    (Value), making oil-stain detection robust across different lighting.
  • We use a running average over several frames to avoid false positives from
    single-frame noise.
  • The OpenCV processing runs on the same node for simplicity; in a production
    system it would be a separate perception pipeline.

IMPORTANT: This script requires the `cv_bridge` and `opencv-python` packages.
"""

import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from arm_control_helpers import ArmController, load_mission_config

# ── Try importing OpenCV ─────────────────────────────────────────────────────
# In a simulation environment OpenCV may not be available; we handle that
# gracefully by providing a stub detection.
try:
    import cv2                                # OpenCV for image processing
    import numpy as np                        # Numerical arrays
    from cv_bridge import CvBridge            # ROS Image ↔ OpenCV Mat converter
    HAS_CV = True
except ImportError:
    HAS_CV = False


class OilLeakDetector(ArmController):
    """
    Extends ArmController to add oil-leak detection via camera images.

    Inherits all arm-motion capabilities and adds:
      • Camera image subscription
      • HSV-based oil detection algorithm
      • Result publishing
    """

    def __init__(self):
        super().__init__(node_name='task2_inspect_damage')

        # ── Detection config from YAML ───────────────────────────────────────
        det_cfg = self.config['oil_leak_detection']
        self.hsv_lower = tuple(det_cfg['hsv_lower'])   # e.g. (0, 0, 0)
        self.hsv_upper = tuple(det_cfg['hsv_upper'])   # e.g. (180, 255, 50)
        self.min_area  = det_cfg['min_contour_area']    # minimum pixel area
        self.camera_topic = det_cfg['camera_topic']
        self.confidence_threshold = det_cfg['detection_confidence']

        # ── State tracking ───────────────────────────────────────────────────
        self.frames_processed = 0        # how many camera frames analysed
        self.positive_detections = 0     # how many had oil-like regions
        self.detection_complete = False  # flag to stop processing

        # ── ROS interfaces ───────────────────────────────────────────────────
        # Publisher: announces True/False for leak detected
        self.leak_pub = self.create_publisher(Bool, '/oil_leak_detected', 10)
        # Publisher: human-readable status string
        self.status_pub = self.create_publisher(
            String, '/inspection_status', 10
        )

        if HAS_CV:
            self.bridge = CvBridge()  # one-time init of the converter
            # Subscribe to camera images
            self.image_sub = self.create_subscription(
                Image,
                self.camera_topic,
                self._image_callback,
                10
            )
            self.get_logger().info(
                f'Subscribed to camera: {self.camera_topic}'
            )
        else:
            self.get_logger().warn(
                'OpenCV/cv_bridge not available — using simulated detection'
            )

    def _image_callback(self, msg):
        """
        Process each incoming camera frame for oil-leak signatures.

        Parameters
        ----------
        msg : sensor_msgs/Image
            Raw camera image in BGR8 encoding.

        Algorithm:
        1. Convert ROS Image → OpenCV BGR matrix via CvBridge
        2. Convert BGR → HSV colour space
        3. Create binary mask where pixels fall within [hsv_lower, hsv_upper]
        4. Find contours in the mask
        5. Filter contours by minimum area (reject noise)
        6. If any large contour exists → positive detection
        """
        if self.detection_complete:
            return  # stop processing after enough frames

        try:
            # Step 1: ROS Image → OpenCV Mat (BGR 8-bit)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Step 2: BGR → HSV
            # HSV = (Hue 0-180, Saturation 0-255, Value 0-255) in OpenCV
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # Step 3: Threshold to isolate dark oil-like regions
            # inRange returns a binary mask: 255 where pixel is in range, 0 otherwise
            mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)

            # Step 4: Find contours (connected regions) in the binary mask
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            # Step 5: Check if any contour is large enough to be an oil stain
            leak_found = any(
                cv2.contourArea(c) > self.min_area for c in contours
            )

            # Step 6: Update running statistics
            self.frames_processed += 1
            if leak_found:
                self.positive_detections += 1

            # After 20 frames, make a decision
            if self.frames_processed >= 20:
                self.detection_complete = True

        except Exception as e:
            self.get_logger().error(f'Image processing error: {e}')

    def run_detection(self, num_frames=20, timeout_sec=10.0):
        """
        Spin the node until enough frames are processed or timeout.

        Parameters
        ----------
        num_frames : int
            Desired number of frames to analyse.
        timeout_sec : float
            Maximum time to wait.

        Returns
        -------
        bool
            True if oil leak detected with sufficient confidence.
        """
        if not HAS_CV:
            # ── Simulated detection for environments without OpenCV ──────────
            self.get_logger().info(
                '🔍 [SIMULATED] Running oil-leak detection...'
            )
            time.sleep(3.0)  # simulate processing time
            self.get_logger().info(
                '🔍 [SIMULATED] Oil leak DETECTED (simulated positive)'
            )
            # Publish detection result
            leak_msg = Bool()
            leak_msg.data = True
            self.leak_pub.publish(leak_msg)
            return True

        # ── Real detection loop ──────────────────────────────────────────────
        start = time.time()
        while not self.detection_complete and (time.time() - start) < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)

        # ── Evaluate results ─────────────────────────────────────────────────
        if self.frames_processed == 0:
            self.get_logger().warn('No camera frames received')
            # Assume leak present for safety (fail-safe)
            confidence = 1.0
        else:
            confidence = self.positive_detections / self.frames_processed

        leak_detected = confidence >= self.confidence_threshold

        # ── Publish ──────────────────────────────────────────────────────────
        leak_msg = Bool()
        leak_msg.data = leak_detected
        self.leak_pub.publish(leak_msg)

        status_msg = String()
        status_msg.data = (
            f'Inspection complete: {self.frames_processed} frames, '
            f'{self.positive_detections} positives, '
            f'confidence={confidence:.2f}, '
            f'leak={"YES" if leak_detected else "NO"}'
        )
        self.status_pub.publish(status_msg)
        self.get_logger().info(status_msg.data)

        return leak_detected


def main():
    """Entry point for Task 2: Inspect Damage."""
    rclpy.init(args=sys.argv)

    detector = OilLeakDetector()
    logger = detector.get_logger()

    logger.info('═══════════════════════════════════════')
    logger.info('  TASK 2: INSPECT DAMAGE – STARTING')
    logger.info('═══════════════════════════════════════')

    try:
        # ── Step 1: Position the arm for inspection ──────────────────────────
        logger.info('Step 1/3 ▶ Moving to INSPECT_READY position')
        detector.move_to_named_pose('inspect_ready', duration_sec=4.0)
        detector.pause('Camera positioning')

        # ── Step 2: Run oil-leak detection ───────────────────────────────────
        logger.info('Step 2/3 ▶ Scanning for oil leaks...')
        leak_found = detector.run_detection()

        # ── Step 3: Report ───────────────────────────────────────────────────
        if leak_found:
            logger.info('🚨 OIL LEAK CONFIRMED — proceeding to repair')
        else:
            logger.info('✅ No significant leak detected')

        logger.info('═══════════════════════════════════════')
        logger.info('  TASK 2: INSPECT DAMAGE – COMPLETE ✓')
        logger.info('═══════════════════════════════════════')

    except Exception as e:
        logger.error(f'Task 2 failed: {e}')

    finally:
        detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
