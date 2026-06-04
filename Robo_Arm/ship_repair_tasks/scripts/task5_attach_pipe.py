#!/usr/bin/env python3
"""
Task 5 – Attach New Pipe
═════════════════════════
The arm connects and secures the replacement pipe to the oxygen system,
ensuring the flow is restored.

Sequence:
  1. Move to HOME (safe transit position)
  2. Open gripper (prepare to grab replacement pipe)
  3. Move to TOOL_PICKUP (where the replacement pipe is stored)
  4. Close gripper (grasp the replacement pipe)
  5. Move to PIPE_ATTACH position (alignment with the oxygen system)
  6. Execute fine-alignment motion (small adjustments)
  7. Open gripper (release the pipe into its seat)
  8. Close gripper on coupling nut and tighten (rotate wrist)
  9. Release and retract

Engineering Rationale:
  • Pipe attachment requires high precision — the arm first does a coarse
    approach, then fine-alignment with small joint increments.
  • The "tightening" is simulated by rotating Joint_6 (wrist roll) while the
    gripper holds the coupling nut.  On real hardware, a torque-limited mode
    would be used to avoid overtightening.
"""

import sys
import time
import rclpy

from arm_control_helpers import ArmController


def main():
    """Entry point for Task 5: Attach New Pipe."""
    rclpy.init(args=sys.argv)

    ctrl = ArmController(node_name='task5_attach_pipe')
    logger = ctrl.get_logger()

    logger.info('═══════════════════════════════════════')
    logger.info('  TASK 5: ATTACH NEW PIPE – STARTING')
    logger.info('═══════════════════════════════════════')

    try:
        # ── Step 1: Safe transit via HOME ────────────────────────────────────
        logger.info('Step 1/8 ▶ Moving to HOME for safe transit')
        ctrl.move_to_named_pose('home', duration_sec=3.0)

        # ── Step 2: Open gripper ─────────────────────────────────────────────
        logger.info('Step 2/8 ▶ Opening gripper to grab replacement pipe')
        ctrl.open_gripper()

        # ── Step 3: Pick up replacement pipe ─────────────────────────────────
        logger.info('Step 3/8 ▶ Moving to TOOL_PICKUP (pipe storage)')
        ctrl.move_to_named_pose('tool_pickup', duration_sec=4.0)
        ctrl.pause('Aligning with pipe')

        # ── Step 4: Grasp pipe ───────────────────────────────────────────────
        logger.info('Step 4/8 ▶ Closing gripper on replacement pipe')
        ctrl.close_gripper()

        # ── Step 5: Coarse approach ──────────────────────────────────────────
        logger.info('Step 5/8 ▶ Moving to PIPE_ATTACH position')
        ctrl.move_to_named_pose('pipe_attach', duration_sec=5.0)
        ctrl.pause('Coarse alignment')

        # ── Step 6: Fine alignment ───────────────────────────────────────────
        # Small joint adjustments to align pipe connector precisely.
        # In production, this would use force/torque feedback.
        logger.info('Step 6/8 ▶ Fine alignment (micro-adjustments)')
        pipe_pose = dict(ctrl.config['named_poses']['pipe_attach'])
        # Nudge wrist pitch to align connectors
        pipe_pose['Joint_5'] = pipe_pose.get('Joint_5', 0.0) + 0.05
        pipe_pose['Joint_6'] = pipe_pose.get('Joint_6', 0.0) + 0.10
        ctrl.move_to_joint_positions(pipe_pose, duration_sec=2.0)
        ctrl.pause('Pipe aligned')

        # ── Step 7: Release pipe into seat ───────────────────────────────────
        logger.info('Step 7/8 ▶ Releasing pipe into connector seat')
        ctrl.open_gripper()
        ctrl.pause('Pipe seated')

        # ── Step 8: Tighten coupling ─────────────────────────────────────────
        # Grab the coupling nut and rotate wrist to tighten
        logger.info('Step 8/8 ▶ Tightening coupling (wrist rotation)')
        ctrl.close_gripper()
        tighten_pose = dict(pipe_pose)
        # Rotate wrist 90° to tighten coupling
        tighten_pose['Joint_6'] = tighten_pose.get('Joint_6', 0.0) + 1.57
        ctrl.move_to_joint_positions(tighten_pose, duration_sec=3.0)
        ctrl.pause('Coupling tightened')

        # Release and retract slightly
        ctrl.release_gripper()

        logger.info('═══════════════════════════════════════')
        logger.info('  TASK 5: ATTACH PIPE – COMPLETE ✓')
        logger.info('═══════════════════════════════════════')

    except Exception as e:
        logger.error(f'Task 5 failed: {e}')

    finally:
        ctrl.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
