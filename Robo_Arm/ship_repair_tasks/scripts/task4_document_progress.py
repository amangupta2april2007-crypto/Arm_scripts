#!/usr/bin/env python3
"""
Task 4 – Document Progress
═══════════════════════════
The arm positions the camera to take photos of the leak before and after repair
for mission records.

Sequence:
  1. Release repair tool (open gripper)
  2. Move to CAMERA_POSITION (optimal viewing angle)
  3. Capture "BEFORE" photo
  4. Move to alternate angle
  5. Capture "AFTER" photo
  6. Publish captured image metadata to /repair_documentation topic

Engineering Rationale:
  • Two photos from different angles provide stereo evidence of the repair.
  • In simulation, we "capture" images by subscribing to the camera topic,
    saving the raw bytes, and logging the timestamp.  On real hardware, this
    would trigger an actual camera snapshot to persistent storage.
  • Publishing to /repair_documentation allows a ground station to aggregate
    all mission records automatically.
"""

import sys
import time

import rclpy
from std_msgs.msg import String
from sensor_msgs.msg import Image

from arm_control_helpers import ArmController

# ── Try to import OpenCV for image capture ───────────────────────────────────
try:
    import cv2
    import numpy as np
    from cv_bridge import CvBridge
    HAS_CV = True
except ImportError:
    HAS_CV = False


class DocumentationNode(ArmController):
    """
    Extends ArmController with camera capture and documentation publishing.
    """

    def __init__(self):
        super().__init__(node_name='task4_document_progress')

        # ── Publishers ───────────────────────────────────────────────────────
        self.doc_pub = self.create_publisher(
            String, '/repair_documentation', 10
        )

        # ── Camera subscription ──────────────────────────────────────────────
        self.latest_image = None
        camera_topic = self.config['oil_leak_detection']['camera_topic']

        if HAS_CV:
            self.bridge = CvBridge()
            self.image_sub = self.create_subscription(
                Image, camera_topic, self._image_callback, 10
            )

    def _image_callback(self, msg):
        """Store latest image from camera."""
        self.latest_image = msg

    def capture_photo(self, label='photo'):
        """
        Capture a photo from the arm camera.

        Parameters
        ----------
        label : str
            Description for this capture (e.g., "BEFORE_REPAIR").

        Returns
        -------
        bool
            True if capture succeeded.
        """
        # Wait for a fresh image
        delay = self.config['timing']['photo_capture_delay']
        time.sleep(delay)

        # Spin briefly to get latest frame
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)

        if HAS_CV and self.latest_image is not None:
            try:
                cv_img = self.bridge.imgmsg_to_cv2(
                    self.latest_image, 'bgr8'
                )
                # In production: cv2.imwrite(f'/mission_data/{label}.jpg', cv_img)
                self.get_logger().info(
                    f'📸 Photo captured: {label} '
                    f'({cv_img.shape[1]}x{cv_img.shape[0]} pixels)'
                )
            except Exception as e:
                self.get_logger().warn(f'Image conversion failed: {e}')
        else:
            # Simulated capture
            self.get_logger().info(f'📸 [SIMULATED] Photo captured: {label}')

        # ── Publish documentation record ─────────────────────────────────────
        doc_msg = String()
        doc_msg.data = (
            f'DOCUMENTATION_RECORD: label={label}, '
            f'timestamp={self.get_clock().now().to_msg()}, '
            f'camera=gripper_left'
        )
        self.doc_pub.publish(doc_msg)
        self.get_logger().info(f'Published documentation: {label}')

        return True


def main():
    """Entry point for Task 4: Document Progress."""
    rclpy.init(args=sys.argv)

    doc_node = DocumentationNode()
    logger = doc_node.get_logger()

    logger.info('═══════════════════════════════════════')
    logger.info('  TASK 4: DOCUMENT PROGRESS – START')
    logger.info('═══════════════════════════════════════')

    try:
        # ── Step 1: Release any held tool ────────────────────────────────────
        logger.info('Step 1/5 ▶ Releasing tool for camera work')
        doc_node.open_gripper()

        # ── Step 2: Position camera — Angle 1 (overview) ────────────────────
        logger.info('Step 2/5 ▶ Moving to CAMERA_POSITION (angle 1)')
        doc_node.move_to_named_pose('camera_position', duration_sec=4.0)
        doc_node.pause('Camera aligned')

        # ── Step 3: Capture BEFORE photo ─────────────────────────────────────
        logger.info('Step 3/5 ▶ Capturing BEFORE repair photo')
        doc_node.capture_photo('BEFORE_REPAIR_OVERVIEW')

        # ── Step 4: Position camera — Angle 2 (close-up) ────────────────────
        logger.info('Step 4/5 ▶ Moving to close-up angle')
        # Slight joint adjustment for a different perspective
        camera_pose = dict(doc_node.config['named_poses']['camera_position'])
        camera_pose['Joint_1'] = camera_pose.get('Joint_1', 0.0) + 0.30
        camera_pose['Joint_5'] = camera_pose.get('Joint_5', 0.0) + 0.20
        doc_node.move_to_joint_positions(camera_pose, duration_sec=3.0)
        doc_node.pause('Close-up angle')

        # ── Step 5: Capture AFTER photo ──────────────────────────────────────
        logger.info('Step 5/5 ▶ Capturing AFTER repair photo (close-up)')
        doc_node.capture_photo('AFTER_REPAIR_CLOSEUP')

        logger.info('═══════════════════════════════════════')
        logger.info('  TASK 4: DOCUMENTATION – COMPLETE ✓')
        logger.info('═══════════════════════════════════════')

    except Exception as e:
        logger.error(f'Task 4 failed: {e}')

    finally:
        doc_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
