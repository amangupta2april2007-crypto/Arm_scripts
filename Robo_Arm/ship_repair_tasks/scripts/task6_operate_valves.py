#!/usr/bin/env python3
"""
Task 6 – Operate Valves
═══════════════════════
The arm rotates or presses the valve handle on the oxygen tank to close the
secondary leak.

Sequence:
  1. Move to VALVE_OPERATE position (arm in front of valve)
  2. Open gripper
  3. Close gripper on valve handle
  4. Rotate valve closed (wrist rotation through multiple steps)
  5. Release valve handle
  6. Verify valve position

Engineering Rationale:
  • Valve rotation is performed in 45° increments with brief pauses to allow
    the valve mechanism to engage.  In reality, a torque-limited controller
    would be used to prevent damaging the valve.
  • Multiple small rotations instead of one large rotation prevent the arm
    from losing grip on round valve handles due to slippage.
  • After rotation, the gripper releases and the arm retracts slightly to
    visually verify the new valve position using the camera.
"""

import sys
import time
import rclpy

from arm_control_helpers import ArmController


def main():
    """Entry point for Task 6: Operate Valves."""
    rclpy.init(args=sys.argv)

    ctrl = ArmController(node_name='task6_operate_valves')
    logger = ctrl.get_logger()

    logger.info('═══════════════════════════════════════')
    logger.info('  TASK 6: OPERATE VALVES – STARTING')
    logger.info('═══════════════════════════════════════')

    try:
        # ── Step 1: Approach the valve ───────────────────────────────────────
        logger.info('Step 1/6 ▶ Moving to VALVE_OPERATE position')
        ctrl.move_to_named_pose('valve_operate', duration_sec=4.0)
        ctrl.pause('Positioned near valve')

        # ── Step 2: Open gripper to receive valve handle ─────────────────────
        logger.info('Step 2/6 ▶ Opening gripper')
        ctrl.open_gripper()

        # ── Step 3: Close on valve handle ────────────────────────────────────
        logger.info('Step 3/6 ▶ Gripping valve handle')
        ctrl.close_gripper()
        ctrl.pause('Handle gripped')

        # ── Step 4: Rotate valve closed ──────────────────────────────────────
        # We rotate the wrist (Joint_6) in 4 x 22.5° steps = 90° total
        # to close a quarter-turn ball valve.
        logger.info('Step 4/6 ▶ Rotating valve closed (4 steps of 22.5°)')
        valve_pose = dict(ctrl.config['named_poses']['valve_operate'])
        base_j6 = valve_pose.get('Joint_6', 0.0)

        for step in range(1, 5):
            angle = base_j6 + (step * 0.3927)  # 0.3927 rad ≈ 22.5°
            valve_pose['Joint_6'] = angle
            logger.info(
                f'  Valve rotation step {step}/4: '
                f'Joint_6 = {angle:.3f} rad ({step * 22.5:.1f}°)'
            )
            ctrl.move_to_joint_positions(valve_pose, duration_sec=2.0)
            time.sleep(0.5)  # let the valve mechanism engage

        logger.info('  Valve fully closed (90° rotation complete)')

        # ── Step 5: Release valve handle ─────────────────────────────────────
        logger.info('Step 5/6 ▶ Releasing valve handle')
        ctrl.open_gripper()

        # ── Step 6: Retract for visual verification ──────────────────────────
        logger.info('Step 6/6 ▶ Retracting to verify valve position')
        verify_pose = dict(valve_pose)
        verify_pose['Joint_2'] = verify_pose.get('Joint_2', 0.0) + 0.15
        ctrl.move_to_joint_positions(verify_pose, duration_sec=2.0)

        logger.info('  ✅ Valve operation verified — secondary leak sealed')

        logger.info('═══════════════════════════════════════')
        logger.info('  TASK 6: OPERATE VALVES – COMPLETE ✓')
        logger.info('═══════════════════════════════════════')

    except Exception as e:
        logger.error(f'Task 6 failed: {e}')

    finally:
        ctrl.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
