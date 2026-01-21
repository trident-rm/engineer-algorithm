from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("robo_arm3", package_name="robo_arm3_moveit_config").to_moveit_configs()
    
    real_world_node = Node(
        package='z_plan',
        executable='real_world',
        name='real_world_node',
        output='screen',
        remappings=[
            ('/usb_joint_states', '/joint_states'),
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
        ],
    )

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[moveit_config.to_dict()],
    )
    
    return LaunchDescription([
        move_group_node,
        real_world_node,
    ])
