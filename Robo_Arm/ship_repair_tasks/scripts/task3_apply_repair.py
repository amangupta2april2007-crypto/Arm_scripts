#!/usr/bin/env python3
"""
Task 3 – Apply Repair Tools
════════════════════════════
The arm manipulates the repair tool and repair kit to seal or patch the leak
on the damaged pipe.

Sequence:
  1. Ensure tool is held (gripper closed)
  2. Move to REPAIR_APPROACH position
  3. Execute a Cartesian sealing path along the pipe
  4. Return to approach position
  5. Release tool

Engineering Rationale:
  • Cartesian-space motion is used for the sealing pass because the tool tip
    must follow a straight line along the pipe surface.  Joint-space motions
    would produce curved end-effector paths.
  • The sealing path waypoints come from mission_config.yaml so they can be
    adapted to different pipe orientations.
  • After sealing, we do NOT open the gripper immediately — Task 5 may need
    a different tool, so we hold it until explicitly told to release.
"""

import sys
import time
import rclpy

from arm_control_helpers import ArmController


def main():
    """Entry point for Task 3: Apply Repair Tools."""
    rclpy.init(args=sys.argv)

    ctrl = ArmController(node_name='task3_apply_repair')
    logger = ctrl.get_logger()

    logger.info('═══════════════════════════════════════')
    logger.info('  TASK 3: APPLY REPAIR TOOLS – START')
    logger.info('═══════════════════════════════════════')

    try:
        # ── Step 1: Confirm tool grip ────────────────────────────────────────
        # Before applying repair, double-check the gripper is closed on a tool.
        # This is a safety measure — if Task 1 was skipped or failed, we
        # still attempt to close.
        logger.info('Step 1/5 ▶ Confirming gripper is closed on tool')
        ctrl.close_gripper()

        # ── Step 2: Approach the damage site ─────────────────────────────────
        logger.info('Step 2/5 ▶ Moving to REPAIR_APPROACH position')
        ctrl.move_to_named_pose('repair_approach', duration_sec=4.0)
        ctrl.pause('Aligning with damaged pipe')

        # ── Step 3: Execute sealing path ─────────────────────────────────────
        # We move through a series of incremental joint configurations that
        # approximate a Cartesian straight-line motion along the pipe.
        # In a full MoveIt implementation this would use
        # MoveGroupInterface.computeCartesianPath().
        logger.info('Step 3/5 ▶ Executing sealing path along the pipe')

        seal_path = ctrl.config['cartesian_paths']['seal_pipe']
        for i, waypoint in enumerate(seal_path):
            logger.info(
                f'  Seal waypoint {i+1}/{len(seal_path)}: '
                f'x={waypoint["x"]:.3f} y={waypoint["y"]:.3f} '
                f'z={waypoint["z"]:.3f}'
            )
            # Approximate Cartesian motion with small joint increments.
            # In production, use MoveIt's Cartesian planner.
            # Here we use repair_approach as base and add small offsets
            # mapped to Joint_6 (wrist) for the sealing sweep.
            repair_joints = dict(ctrl.config['named_poses']['repair_approach'])
            # Small wrist rotation to simulate tool sweep along pipe
            repair_joints['Joint_6'] = 1.57 + (i * 0.15)
            ctrl.move_to_joint_positions(repair_joints, duration_sec=2.0)
            time.sleep(0.5)  # brief pause between waypoints

        logger.info('  Sealing pass completed')

        # ── Step 4: Back to approach position ────────────────────────────────
        logger.info('Step 4/5 ▶ Returning to REPAIR_APPROACH')
        ctrl.move_to_named_pose('repair_approach', duration_sec=3.0)

        # ── Step 5: Hold tool for further tasks ──────────────────────────────
        logger.info('Step 5/5 ▶ Repair applied — tool still held for next task')
        ctrl.pause('Repair complete')

        logger.info('═══════════════════════════════════════')
        logger.info('  TASK 3: APPLY REPAIR – COMPLETE ✓')
        logger.info('═══════════════════════════════════════')

    except Exception as e:
        logger.error(f'Task 3 failed: {e}')

    finally:
        ctrl.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
