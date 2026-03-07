from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("robo_arm6", package_name="robo_arm6_moveit_config").to_moveit_configs()
    
    real_world_node = Node(
        package='z_plan',
        executable='real_world',
        name='real_world_node',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': False}
        ],
    )

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': False}
        ],
    )
    # Robot State Publisher (publish TF from robot_description)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[moveit_config.robot_description],
    )

    # RViz with MoveIt config (if the moveit.rviz exists in the moveit config package)
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
        robot_state_publisher,
        move_group_node,
        rviz,
        real_world_node,
    ])
