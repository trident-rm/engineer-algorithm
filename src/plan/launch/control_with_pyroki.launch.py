"""
control_with_pyroki.launch.py

用法:
  ros2 launch plan control_with_pyroki.launch.py action:=1
    1=UP  2=DOWN  3=LEFT  4=RIGHT  5=TURN

需要先在另一个终端跑:
  ros2 launch plan pyroki_ik.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("robo_arm10", package_name="robo_arm10_moveit_config")
        .to_moveit_configs()
    )

    action_arg = DeclareLaunchArgument(
        "action",
        default_value="1",
        description="1=UP 2=DOWN 3=LEFT 4=RIGHT 5=TURN",
    )

    control_node = Node(
        package="plan",
        executable="control_with_pyroki",
        name="control_with_pyroki",
        output="screen",
        arguments=[LaunchConfiguration("action")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription([action_arg, control_node])
