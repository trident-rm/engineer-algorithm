"""
flow_with_pyroki.launch.py
需要先在另一个终端跑:
  python3 ./src/plan/scripts/pyroki_ik_node.py
  ros2 launch plan pyroki_ik.launch.py
然后:
  ros2 launch plan flow_with_pyroki.launch.py
键盘输入要在启动它的终端里, 焦点放上面按 w / q.
"""
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("robo_arm10", package_name="robo_arm10_moveit_config").to_moveit_configs()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description, {'use_sim_time': False}],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description, {'use_sim_time': False}],
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {'use_sim_time': False}],
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
            {'use_sim_time': False}
        ],
    )

    flow_with_pyroki_node = Node(
        package="plan", 
        executable="flow_with_pyroki",
        name="flow_with_pyroki",
        output="screen",            
        emulate_tty=True,           # 让输出像在终端一样，支持颜色等格式
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {'use_sim_time': False}
        ],
    )
    return LaunchDescription([
        robot_state_publisher,
        # joint_state_publisher,
        move_group_node,
        flow_with_pyroki_node,
        rviz,
    
    ])