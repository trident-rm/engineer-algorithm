"""
flow_with_pyroki.launch.py
需要先在另一个终端跑:
  python3 ./src/plan/scripts/pyroki_ik_node.py
  ros2 launch plan pyroki_ik.launch.py
然后:
  ros2 launch plan flow_with_pyroki.launch.py
键盘输入要在启动它的终端里, 焦点放上面按 w / q.
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("robo_arm10", package_name="robo_arm10_moveit_config")
        .to_moveit_configs()
    )
    return LaunchDescription([
        Node(
            package="plan",
            executable="flow_with_pyroki",
            name="flow_with_pyroki",
            output="screen",
            emulate_tty=True,
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
            ],
        ),
    ])