import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import (
    generate_static_virtual_joint_tfs_launch,
    generate_rsp_launch,
    generate_move_group_launch,
)
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # 构建 MoveIt 配置
    moveit_config = (
        MoveItConfigsBuilder("robo_arm10", package_name="robo_arm10_moveit_config")
        .robot_description(file_path="config/robo_arm10.urdf.xacro")
        .robot_description_semantic(file_path="config/robo_arm10.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )
    
    # Generate static virtual joint TFs
    static_virtual_joint_tfs = generate_static_virtual_joint_tfs_launch(moveit_config)
    
    # Use standard MoveIt launch generators
    rsp_launch = generate_rsp_launch(moveit_config)
    move_group_launch = generate_move_group_launch(moveit_config)
    
    # real_world_new 节点
    real_world_node = Node(
        package='plan',
        executable='real_world_new',
        name='real_world_node',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {
                'use_sim_time': False,
            }
        ],
    )
    
    # Joint state publisher to provide initial state
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': moveit_config.robot_description,
        }],
    )
    
    rviz_config_file = os.path.join(
        get_package_share_directory('robo_arm10_moveit_config'),
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
    
    return LaunchDescription(
        list(static_virtual_joint_tfs.entities) +
        list(rsp_launch.entities) +
        list(move_group_launch.entities) +
        [
            joint_state_publisher,
            rviz,
            real_world_node,
        ]
    )
    
