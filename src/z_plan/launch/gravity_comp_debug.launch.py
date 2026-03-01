"""
Gravity Compensation Debug Launch File

Reads joint angles from serial, shows robot in RViz,
prints Pinocchio vs Generated C torque comparison to terminal.

Usage:
  ros2 launch z_plan gravity_comp_debug.launch.py
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("robo_arm6", package_name="robo_arm6_moveit_config")
        .robot_description(file_path="config/robo_arm6.urdf.xacro")
        .robot_description_semantic(file_path="config/robo_arm6.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

    debug_node = Node(
        package='z_plan',
        executable='gravity_comp_debug',
        name='gravity_comp_debug',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': False},
        ],
    )

    movegroup_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': False},
        ],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[moveit_config.robot_description],
    )

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

    return LaunchDescription([
        rviz,
        robot_state_publisher,
        movegroup_node,
        debug_node,
    ])
