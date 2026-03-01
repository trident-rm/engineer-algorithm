"""
约束控制测试环境 Launch 文件

不依赖串口/硬件，纯仿真验证雅可比 + e_hat 平面约束。

用法:
  ros2 launch z_plan test_constraint.launch.py
  ros2 launch z_plan test_constraint.launch.py test_mode:=random
  ros2 launch z_plan test_constraint.launch.py test_mode:=manual

测试模式:
  circle  - controller 绕圆运动（默认，会产生e_hat方向分量来测试约束）
  random  - controller 随机运动
  manual  - 通过 /test_controller_joints topic 手动设置 controller 关节角

手动模式示例:
  ros2 topic pub /test_controller_joints sensor_msgs/msg/JointState \
    "{position: [0.3, 1.5, -0.5, 1.5, 1.5, 0.0]}" --once
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch 参数
    test_mode_arg = DeclareLaunchArgument(
        'test_mode',
        default_value='circle',
        description='Test mode: circle, random, or manual'
    )

    # MoveIt 配置
    moveit_config = (
        MoveItConfigsBuilder("robo_arm6", package_name="robo_arm6_moveit_config")
        .robot_description(file_path="config/robo_arm6.urdf.xacro")
        .robot_description_semantic(file_path="config/robo_arm6.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

    # 测试节点
    test_node = Node(
        package='z_plan',
        executable='test_constraint',
        name='test_constraint',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {
                'use_sim_time': False,
                'test_mode': LaunchConfiguration('test_mode'),
            }
        ],
    )

    # MoveGroup（测试节点需要 MoveGroupInterface）
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

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[moveit_config.robot_description],
    )

    # RViz
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
        test_mode_arg,
        rviz,
        robot_state_publisher,
        movegroup_node,
        test_node,
    ])
