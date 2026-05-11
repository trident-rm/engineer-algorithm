"""
pyroki_ik.launch.py
启动:
  - robot_state_publisher  (订阅 /joint_states 发布 TF)
  - move_group             (MoveIt 主节点)
  - rviz2                  (加载 MoveIt 配置)
  - pyroki_ik_node.py      (Python 节点: viser 网页 + IK + 发布 /joint_states)
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("robo_arm10", package_name="robo_arm10_moveit_config")
        .to_moveit_configs()
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    rviz_config_file = os.path.join(
        get_package_share_directory("robo_arm10_moveit_config"),
        "config",
        "moveit.rviz",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )


    return LaunchDescription(
        [
            robot_state_publisher,
            move_group_node,
            rviz,
        ]
    )
