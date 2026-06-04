#!/usr/bin/env python3
"""
ship_repair_mission.launch.py
════════════════════════════════
Brings up the full ship-repair pipeline:
  1. Gazebo simulation (empty world)
  2. Robot State Publisher (URDF → /robot_description)
  3. Spawn robot entity in Gazebo
  4. ros2_control controllers (arm + gripper + joint_state_broadcaster)
  5. MoveIt move_group node (optional, for interactive planning)
  6. RViz with MoveIt plugin
  7. ros_gz_bridge (clock sync)

All with proper event-based sequencing so controllers start AFTER
the robot is spawned, and MoveIt starts AFTER controllers are loaded.
"""

import os
import yaml
import xacro

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def load_file(package_name, file_path):
    """Load a file from an installed ROS 2 package share directory."""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, 'r') as f:
        return f.read()


def load_yaml(package_name, file_path):
    """Load a YAML file from an installed ROS 2 package share directory."""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    """Generate the full launch description for ship repair mission."""

    robot_pkg = 'my_robotic_arm'
    moveit_pkg = 'my_robotic_arm_moveit_config'

    # ── Robot Description (process xacro → URDF XML) ─────────────────────────
    controllers_file_path = os.path.join(
        get_package_share_directory(robot_pkg),
        'config', 'my_controllers.yaml'
    )
    urdf_path = os.path.join(
        get_package_share_directory(robot_pkg), 'urdf', 'arm_urdf.urdf'
    )
    doc = xacro.process_file(
        urdf_path, mappings={'controllers_yaml': controllers_file_path}
    )
    robot_description_content = doc.toxml()
    robot_description = {'robot_description': robot_description_content}

    # ── SRDF ─────────────────────────────────────────────────────────────────
    srdf_content = load_file(moveit_pkg, 'config/arm_urdf.srdf')
    robot_description_semantic = {
        'robot_description_semantic': srdf_content
    }

    # ── Kinematics ───────────────────────────────────────────────────────────
    kinematics_yaml = load_yaml(moveit_pkg, 'config/kinematics.yaml')

    # ── OMPL Planning ────────────────────────────────────────────────────────
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters':
                'default_planner_request_adapters/AddTimeOptimalParameterization '
                'default_planner_request_adapters/FixWorkspaceBounds '
                'default_planner_request_adapters/FixStartStateBounds '
                'default_planner_request_adapters/FixStartStateCollision '
                'default_planner_request_adapters/ResolveConstraintFrames',
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_yaml = load_yaml(moveit_pkg, 'config/ompl_planning.yaml')
    if ompl_yaml:
        ompl_planning_pipeline_config['move_group'].update(ompl_yaml)

    # ── Trajectory Execution ─────────────────────────────────────────────────
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    # ── MoveIt Controllers ───────────────────────────────────────────────────
    moveit_controllers_yaml = load_yaml(
        moveit_pkg, 'config/moveit_controllers.yaml'
    )
    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_controllers_yaml,
        'moveit_controller_manager':
            'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

    # ══════════════════════════════════════════════════════════════════════════
    # NODES
    # ══════════════════════════════════════════════════════════════════════════

    # 1. Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # 2. Robot State Publisher
    node_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    # 3. Spawn Robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'my_robotic_arm',
            '-z', '0.1',
        ],
        output='screen',
    )

    # 4. Clock Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen',
    )

    # 5. Controller Spawners
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager'
        ],
    )

    arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager', '/controller_manager'
        ],
    )

    gripper_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'gripper_controller',
            '--controller-manager', '/controller_manager'
        ],
    )

    # 6. MoveIt Move Group
    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            {'use_sim_time': True},
        ],
    )

    # 7. RViz
    rviz_config = os.path.join(
        get_package_share_directory(moveit_pkg), 'config', 'moveit.rviz'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        arguments=['-d', rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
            {'use_sim_time': True},
        ],
    )

    # ══════════════════════════════════════════════════════════════════════════
    # EVENT-BASED EXECUTION ORDER
    # ══════════════════════════════════════════════════════════════════════════
    return LaunchDescription([
        # Phase 1: Start Gazebo + RSP + Bridge immediately
        gazebo,
        bridge,
        node_rsp,

        # Phase 2: Spawn robot (after Gazebo)
        spawn_entity,

        # Phase 3: Joint State Broadcaster (after spawn completes)
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[jsb_spawner],
            )
        ),

        # Phase 4: Arm controller (after JSB)
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=jsb_spawner,
                on_exit=[arm_spawner],
            )
        ),

        # Phase 5: Gripper controller (after arm controller)
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=arm_spawner,
                on_exit=[gripper_spawner],
            )
        ),

        # Phase 6: MoveIt + RViz (after all controllers)
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gripper_spawner,
                on_exit=[run_move_group_node, rviz_node],
            )
        ),
    ])
