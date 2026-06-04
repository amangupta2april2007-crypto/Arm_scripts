#!/usr/bin/env python3
"""
Task 1 – Deliver Tools
══════════════════════
The arm picks up and carries the Emergency Tool and Repair Kit from the rover
to the astronaut's location.

Sequence:
  1. Move to HOME position (safe starting pose)
  2. Open gripper (prepare to grasp)
  3. Move to TOOL_PICKUP position (above the tool rack)
  4. Close gripper (grasp the tool)
  5. Move to REPAIR_APPROACH position (deliver near damage site)
  6. Log completion

Engineering Rationale:
  • We open the gripper BEFORE approaching the tool to avoid collisions.
  • We use named_poses from the YAML so operators can adjust positions in the
    field without recompiling.
"""

# ─── Standard Library ────────────────────────────────────────────────────────
import sys          # sys.argv is forwarded to rclpy.init()

# ─── ROS 2 ───────────────────────────────────────────────────────────────────
import rclpy        # rclpy.init() / rclpy.shutdown() bracket all ROS 2 use

# ─── Project Helpers ─────────────────────────────────────────────────────────
# We import from the installed package; colcon copies arm_control_helpers.py
# into install/lib/ship_repair_tasks/ alongside this script.
# Because both are in the same directory at runtime, a simple import works.
from arm_control_helpers import ArmController


def main():
    """
    Entry point for Task 1: Deliver Tools.

    This function is called when you run:
        ros2 run ship_repair_tasks task1_deliver_tools.py
    """
    # ── 1. Initialise ROS 2 ──────────────────────────────────────────────────
    # rclpy.init() sets up the ROS 2 middleware (DDS) and parses any ROS args
    # from the command line (e.g., --ros-args -r __node:=my_node).
    rclpy.init(args=sys.argv)

    # ── 2. Create our controller node ────────────────────────────────────────
    ctrl = ArmController(node_name='task1_deliver_tools')
    logger = ctrl.get_logger()  # shorthand for logging

    logger.info('═══════════════════════════════════════')
    logger.info('  TASK 1: DELIVER TOOLS – STARTING')
    logger.info('═══════════════════════════════════════')

    try:
        # ── Step 1: Go to home position ──────────────────────────────────────
        # The home pose is all-zeros, meaning the arm is straight up.
        # This is the safest starting posture because it has maximum clearance
        # from any obstacles.
        logger.info('Step 1/5 ▶ Moving to HOME position')
        ctrl.move_to_named_pose('home', duration_sec=3.0)
        ctrl.pause('Settling at home')

        # ── Step 2: Open gripper ─────────────────────────────────────────────
        # We must open the gripper BEFORE moving to the tool, otherwise the
        # closed fingers could collide with the tool holder.
        logger.info('Step 2/5 ▶ Opening gripper for tool pickup')
        ctrl.open_gripper()

        # ── Step 3: Move to tool pickup ──────────────────────────────────────
        # tool_pickup pose places the end-effector directly above the tool rack
        # on the rover.  The exact angles are in mission_config.yaml.
        logger.info('Step 3/5 ▶ Moving to TOOL PICKUP position')
        ctrl.move_to_named_pose('tool_pickup', duration_sec=4.0)
        ctrl.pause('Aligning with tool')

        # ── Step 4: Grasp the tool ───────────────────────────────────────────
        # close_gripper() commands Finger_1 and Finger_2 to the "closed"
        # positions from the config (5 mm each side), firmly gripping the tool.
        logger.info('Step 4/5 ▶ Closing gripper – grasping tool')
        ctrl.close_gripper()

        # ── Step 5: Carry tool to repair site ────────────────────────────────
        # repair_approach is a joint configuration that positions the arm near
        # the astronaut / damage site while keeping the tool safely held.
        logger.info('Step 5/5 ▶ Carrying tool to REPAIR APPROACH position')
        ctrl.move_to_named_pose('repair_approach', duration_sec=5.0)
        ctrl.pause('Tool delivered')

        logger.info('═══════════════════════════════════════')
        logger.info('  TASK 1: DELIVER TOOLS – COMPLETE ✓')
        logger.info('═══════════════════════════════════════')

    except Exception as e:
        # Catch any MoveIt / action-client errors and log them cleanly.
        logger.error(f'Task 1 failed: {e}')

    finally:
        # ── Cleanup ──────────────────────────────────────────────────────────
        # Always destroy the node and shut down rclpy, even on errors.
        # This releases DDS resources and avoids zombie processes.
        ctrl.destroy_node()
        rclpy.shutdown()


# ── Guard: only run main() when executed as a script, not when imported ──────
if __name__ == '__main__':
    main()
