"""
Launch file for real_world_servo node with MoveIt Servo integration.

替代 real_world_new.launch.py 中基于 computeCartesianPath 的实时控制，
使用 MoveIt Servo 通过增量 Twist 指令实现平面约束的实时伺服控制。
"""
import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    """加载 yaml 配置文件"""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    # 构建 MoveIt 配置
    moveit_config = (
        MoveItConfigsBuilder("robo_arm6", package_name="robo_arm6_moveit_config")
        .robot_description(file_path="config/robo_arm6.urdf.xacro")
        .robot_description_semantic(file_path="config/robo_arm6.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

    # 加载 Servo 配置
    servo_yaml = load_yaml("robo_arm6_moveit_config", "config/servo_config.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    # real_world_servo 节点 — 替代 real_world_new
    real_world_servo_node = Node(
        package='z_plan',
        executable='real_world_servo',
        name='real_world_servo_node',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            servo_params,
            {'use_sim_time': False},
        ],
    )

    # MoveGroup 节点（仍然需要，用于 FK/模型加载等）
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
        rviz,
        robot_state_publisher,
        movegroup_node,
        real_world_servo_node,
    ])
