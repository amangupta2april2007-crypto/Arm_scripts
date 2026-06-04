#!/usr/bin/env python3
"""
Task 7 – Return Tools
═════════════════════
After completing repairs, the arm stows the tools back on the rover before
returning to base.

Sequence:
  1. Move to current holding position (whatever pose the arm is in)
  2. If gripper is holding something → pick up, else open
  3. Move to TOOL_STOW position (above the tool rack)
  4. Open gripper to deposit tools
  5. Retract arm slightly
  6. Return to HOME position (safe travel posture)

Engineering Rationale:
  • We always return to HOME at the end so the arm is in a known, safe
    configuration for the next mission cycle or for transport.
  • The tool_stow position is slightly different from tool_pickup — the stow
    position has a gentler approach angle to avoid knocking tools off the rack.
  • A small retraction after releasing the tool prevents the arm from fouling
    the tool rack when moving to HOME.
"""

import sys
import time
import rclpy

from arm_control_helpers import ArmController


def main():
    """Entry point for Task 7: Return Tools."""
    rclpy.init(args=sys.argv)

    ctrl = ArmController(node_name='task7_return_tools')
    logger = ctrl.get_logger()

    logger.info('═══════════════════════════════════════')
    logger.info('  TASK 7: RETURN TOOLS – STARTING')
    logger.info('═══════════════════════════════════════')

    try:
        # ── Step 1: Ensure we have a tool to return ──────────────────────────
        # Try to close gripper — if a tool is present, this re-grips it;
        # if not, it simply closes on empty air (harmless).
        logger.info('Step 1/5 ▶ Confirming grip on any held tools')
        ctrl.close_gripper()
        ctrl.pause('Grip confirmed')

        # ── Step 2: Move to tool stow position ──────────────────────────────
        logger.info('Step 2/5 ▶ Moving to TOOL_STOW position')
        ctrl.move_to_named_pose('tool_stow', duration_sec=4.0)
        ctrl.pause('Aligned with tool rack')

        # ── Step 3: Deposit tools ────────────────────────────────────────────
        logger.info('Step 3/5 ▶ Opening gripper to deposit tools')
        ctrl.open_gripper()
        ctrl.pause('Tools deposited')

        # ── Step 4: Retract from tool rack ───────────────────────────────────
        # Small retraction to clear the tool before large motion to HOME.
        logger.info('Step 4/5 ▶ Retracting from tool rack')
        stow_pose = dict(ctrl.config['named_poses']['tool_stow'])
        stow_pose['Joint_2'] = stow_pose.get('Joint_2', 0.0) + 0.20
        ctrl.move_to_joint_positions(stow_pose, duration_sec=2.0)
        ctrl.pause('Retracted')

        # ── Step 5: Return to HOME ───────────────────────────────────────────
        logger.info('Step 5/5 ▶ Returning to HOME position')
        ctrl.move_to_named_pose('home', duration_sec=4.0)
        ctrl.close_gripper()  # park gripper in closed position

        logger.info('═══════════════════════════════════════')
        logger.info('  TASK 7: RETURN TOOLS – COMPLETE ✓')
        logger.info('═══════════════════════════════════════')
        logger.info('  🏠 ALL TASKS COMPLETE – ARM PARKED')

    except Exception as e:
        logger.error(f'Task 7 failed: {e}')

    finally:
        ctrl.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
