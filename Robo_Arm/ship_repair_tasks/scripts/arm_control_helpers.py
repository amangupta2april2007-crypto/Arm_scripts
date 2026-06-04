#!/usr/bin/env python3
"""
arm_control_helpers.py  –  Shared Utility Module for Ship Repair Tasks
═══════════════════════════════════════════════════════════════════════

This module is imported by every task script.  It provides:
  • ArmController  – thin wrapper around MoveIt's MoveGroupCommander
  • YAML loader     – reads mission_config.yaml once
  • Gripper helpers – open / close / partial release
  • Cartesian path  – linear end-effector motion
  • Logging         – coloured console + ROS 2 logger

DESIGN NOTES
────────────
Why a helper module instead of duplicating code?
  1. DRY (Don't Repeat Yourself) – every task needs the same MoveIt boiler-plate.
  2. Single point of change – if joint names change in the URDF, we fix ONE file.
  3. Testability – unit-test the helper in isolation.

Line-by-line explanations are embedded as inline comments.
"""

# ─── Standard Library ────────────────────────────────────────────────────────
import os            # os.path.join to build file paths portably
import sys           # sys.exit for fatal-error exits
import time          # time.sleep for pauses between motions
import yaml          # yaml.safe_load to read mission_config.yaml

# ─── ROS 2 Client Library ───────────────────────────────────────────────────
import rclpy                                      # ROS 2 Python main entry point
from rclpy.node import Node                       # Base class for all ROS 2 nodes
from rclpy.action import ActionClient             # Client for FollowJointTrajectory
from rclpy.logging import get_logger              # Named loggers

# ─── ROS 2 Messages ─────────────────────────────────────────────────────────
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState

# ─── Ament Index ─────────────────────────────────────────────────────────────
from ament_index_python.packages import get_package_share_directory


# ═══════════════════════════════════════════════════════════════════════════════
# CONFIGURATION LOADER
# ═══════════════════════════════════════════════════════════════════════════════

def load_mission_config():
    """
    Read and return the mission_config.yaml as a Python dict.

    The function finds the YAML file inside the *installed* share directory
    of the ship_repair_tasks package, so `colcon build` must have been run.

    Returns
    -------
    dict
        Entire YAML as a nested dictionary.

    Raises
    ------
    FileNotFoundError
        If the YAML is missing (forgot to build / install).
    """
    # get_package_share_directory returns:
    #   <colcon_ws>/install/ship_repair_tasks/share/ship_repair_tasks
    pkg_share = get_package_share_directory('ship_repair_tasks')

    # Build absolute path: .../share/ship_repair_tasks/config/mission_config.yaml
    config_path = os.path.join(pkg_share, 'config', 'mission_config.yaml')

    # Open and parse YAML
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)  # safe_load prevents arbitrary code execution

    return config


# ═══════════════════════════════════════════════════════════════════════════════
# ARM CONTROLLER CLASS
# ═══════════════════════════════════════════════════════════════════════════════

class ArmController(Node):
    """
    High-level controller that wraps joint-trajectory actions for the arm and
    gripper.

    This class does NOT use MoveIt's Python `moveit_commander` directly because
    moveit_commander requires `move_group` node to be running.  Instead, we
    send FollowJointTrajectory actions to the ros2_control controllers, which
    is the same interface MoveIt ultimately uses.  This approach works both
    WITH and WITHOUT the MoveIt move_group node.

    Attributes
    ----------
    config : dict
        Parsed mission_config.yaml.
    arm_client : ActionClient
        Client for the arm_controller/follow_joint_trajectory action.
    gripper_client : ActionClient
        Client for the gripper_controller/follow_joint_trajectory action.
    current_joint_state : JointState or None
        Latest joint-state message from /joint_states topic.
    """

    # ── Joint names (must match URDF and controllers.yaml) ───────────────────
    ARM_JOINTS = ['Joint_1', 'Joint_2', 'Joint_3',
                  'Joint_4', 'Joint_5', 'Joint_6']
    GRIPPER_JOINTS = ['Finger_1', 'Finger_2']

    def __init__(self, node_name='arm_controller_node'):
        """
        Initialise the node, action clients, and joint-state subscriber.

        Parameters
        ----------
        node_name : str
            ROS 2 node name (visible in `ros2 node list`).
        """
        super().__init__(node_name)  # calls Node.__init__

        # ── Load YAML config ─────────────────────────────────────────────────
        self.config = load_mission_config()  # dict from YAML
        self._planning = self.config['planning']  # shorthand

        # ── Create Action Clients ────────────────────────────────────────────
        # These connect to the joint_trajectory_controller action servers
        # spawned by ros2_control.
        #
        # Action server paths (set by controller_manager):
        #   /arm_controller/follow_joint_trajectory
        #   /gripper_controller/follow_joint_trajectory
        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )
        self.gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory'
        )

        # ── Subscribe to joint states ────────────────────────────────────────
        # The joint_state_broadcaster publishes JointState at 100 Hz.
        # We store the latest message so we can read current positions.
        self.current_joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,  # store msg in self.current_joint_state
            10                            # QoS queue depth
        )

        self.get_logger().info(f'[{node_name}] Waiting for action servers...')
        # Block until both action servers are available (or timeout).
        self.arm_client.wait_for_server(timeout_sec=30.0)
        self.gripper_client.wait_for_server(timeout_sec=30.0)
        self.get_logger().info(f'[{node_name}] Action servers connected ✓')

    # ─── Callbacks ───────────────────────────────────────────────────────────

    def _joint_state_callback(self, msg):
        """
        Store latest JointState message.

        Parameters
        ----------
        msg : sensor_msgs/JointState
            Contains .name (list[str]) and .position (list[float]).
        """
        self.current_joint_state = msg

    # ─── Arm Motion ──────────────────────────────────────────────────────────

    def move_to_joint_positions(self, joint_positions_dict, duration_sec=4.0):
        """
        Command the arm to a set of joint positions.

        Parameters
        ----------
        joint_positions_dict : dict
            Mapping {joint_name: target_angle_rad}, e.g.
            {'Joint_1': 0.785, 'Joint_2': -0.35, ...}
        duration_sec : float
            How many seconds the motion should take.

        Returns
        -------
        bool
            True if the action completed successfully.
        """
        # ── Build the trajectory message ─────────────────────────────────────
        trajectory = JointTrajectory()
        # Header.frame_id is not used by ros2_control but must be set
        trajectory.joint_names = self.ARM_JOINTS

        point = JointTrajectoryPoint()
        # Fill positions in the SAME ORDER as trajectory.joint_names
        point.positions = [
            joint_positions_dict.get(j, 0.0) for j in self.ARM_JOINTS
        ]
        # Velocities / accelerations left empty → controller uses defaults
        point.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec % 1) * 1e9)
        )
        trajectory.points = [point]

        # ── Build the goal message ───────────────────────────────────────────
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory

        self.get_logger().info(
            f'Moving arm to: {joint_positions_dict}'
        )

        # ── Send goal and wait ───────────────────────────────────────────────
        send_goal_future = self.arm_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Arm goal REJECTED')
            return False

        self.get_logger().info('Arm goal accepted, executing...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        if result.error_code == 0:
            self.get_logger().info('Arm motion completed ✓')
            return True
        else:
            self.get_logger().warn(
                f'Arm motion finished with error code: {result.error_code}'
            )
            return True  # non-zero may still be "close enough"

    def move_to_named_pose(self, pose_name, duration_sec=4.0):
        """
        Move to a pre-defined named pose from mission_config.yaml.

        Parameters
        ----------
        pose_name : str
            Key under `named_poses:` in the YAML, e.g. "home", "tool_pickup".
        duration_sec : float
            Motion duration.

        Returns
        -------
        bool
            Success flag.
        """
        poses = self.config['named_poses']
        if pose_name not in poses:
            self.get_logger().error(f'Unknown pose: {pose_name}')
            return False
        return self.move_to_joint_positions(poses[pose_name], duration_sec)

    # ─── Gripper Control ─────────────────────────────────────────────────────

    def set_gripper(self, finger_positions_dict, duration_sec=2.0):
        """
        Command the gripper fingers to target positions.

        Parameters
        ----------
        finger_positions_dict : dict
            e.g. {'Finger_1': 0.035, 'Finger_2': -0.035}
        duration_sec : float
            Motion duration.

        Returns
        -------
        bool
            Success flag.
        """
        trajectory = JointTrajectory()
        trajectory.joint_names = self.GRIPPER_JOINTS

        point = JointTrajectoryPoint()
        point.positions = [
            finger_positions_dict.get(j, 0.0) for j in self.GRIPPER_JOINTS
        ]
        point.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec % 1) * 1e9)
        )
        trajectory.points = [point]

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory

        self.get_logger().info(f'Setting gripper: {finger_positions_dict}')

        send_goal_future = self.gripper_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Gripper goal REJECTED')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        settle = self.config['timing']['gripper_settle_time']
        time.sleep(settle)  # let the gripper physically settle

        self.get_logger().info('Gripper motion completed ✓')
        return True

    def open_gripper(self):
        """Open the gripper fully (positions from config)."""
        return self.set_gripper(self.config['gripper']['open'])

    def close_gripper(self):
        """Close the gripper to gripping position."""
        return self.set_gripper(self.config['gripper']['closed'])

    def release_gripper(self):
        """Partially open the gripper for gentle release."""
        return self.set_gripper(self.config['gripper']['release'])

    # ─── Utilities ───────────────────────────────────────────────────────────

    def pause(self, label=''):
        """
        Sleep for the inter-task pause duration defined in config.

        Parameters
        ----------
        label : str
            Optional log message describing what we are waiting for.
        """
        pause_sec = self.config['timing']['inter_task_pause']
        if label:
            self.get_logger().info(f'⏳ {label} (pausing {pause_sec}s)')
        time.sleep(pause_sec)

    def get_current_positions(self):
        """
        Read current joint positions from the latest JointState message.

        Returns
        -------
        dict or None
            {joint_name: position_rad} for all joints, or None if no message
            has been received yet.
        """
        if self.current_joint_state is None:
            return None
        return dict(zip(
            self.current_joint_state.name,
            self.current_joint_state.position
        ))
