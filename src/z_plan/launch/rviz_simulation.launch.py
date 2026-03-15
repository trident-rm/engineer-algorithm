"""
RViz-based simulation control for robotic arm with interactive markers.

This is a convenience launch file that defaults to robo_arm6.
For other robots, use the specific launch files:
  - rviz_simulation_robo_arm6.launch.py  (robo_arm6)
  - rviz_simulation_robo_arm3.launch.py  (robo_arm3)
  - rviz_simulation_panda.launch.py      (Panda robot)

Usage:
    # Default (robo_arm6):
    ros2 launch z_plan rviz_simulation.launch.py
    
    # Other robots:
    ros2 launch z_plan rviz_simulation_robo_arm6.launch.py
    ros2 launch z_plan rviz_simulation_robo_arm3.launch.py
    ros2 launch z_plan rviz_simulation_panda.launch.py
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ============================================================================
    # MoveIt Configuration for robo_arm6 (default)
    # ============================================================================
    moveit_config = (
        MoveItConfigsBuilder("robo_arm6", package_name="robo_arm6_moveit_config")
        .robot_description(file_path="config/robo_arm6.urdf.xacro")
        .robot_description_semantic(file_path="config/robo_arm6.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )
    
    # ============================================================================
    # Robot State Publisher
    # ============================================================================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[moveit_config.robot_description],
    )
    
    # ============================================================================
    # MoveGroup Node
    # ============================================================================
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': False},
        ],
    )
    
    # ============================================================================
    # RViz Node
    # ============================================================================
    rviz_config_file = os.path.join(
        get_package_share_directory('robo_arm6_moveit_config'),
        'config',
        'moveit.rviz'
    )
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )
    
    # ============================================================================
    # RViz Simulation Node
    # ============================================================================
    rviz_simulation = Node(
        package='z_plan',
        executable='rviz_simulation',
        name='rviz_simulation',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {'control_mode': 0},  # 初始模式: 0=NO, 1=UP, 2=TURN, 3=ROLL, 4=RESET
        ],
    )
    
    return LaunchDescription([
        robot_state_publisher,
        move_group_node,
        rviz,
        rviz_simulation,
    ])
