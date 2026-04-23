"""
重力补偿对比测试 Launch 文件

对比 Pinocchio RNEA 与生成的 C 代码的重力补偿力矩输出。
不需要 RViz，纯数值测试。

用法:
  ros2 launch z_plan test_gravity_comp.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # MoveIt 配置
    moveit_config = (
        MoveItConfigsBuilder("robo_arm6", package_name="robo_arm6_moveit_config")
        .robot_description(file_path="config/robo_arm6.urdf.xacro")
        .robot_description_semantic(file_path="config/robo_arm6.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[moveit_config.robot_description],
    )

    # MoveGroup (needed for MoveGroupInterface)
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

    # Test node
    test_node = Node(
        package='plan',
        executable='test_gravity_comp',
        name='test_gravity_comp',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': False},
        ],
    )

    return LaunchDescription([
        robot_state_publisher,
        movegroup_node,
        test_node,
    ])
