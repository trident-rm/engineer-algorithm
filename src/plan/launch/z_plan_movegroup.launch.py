from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Use the panda_moveit_config package
    moveit_config = MoveItConfigsBuilder("robo_arm10", package_name="robo_arm10_moveit_config").to_moveit_configs()
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[moveit_config.robot_description],
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
    z_plan_movegroup = Node(
        package='plan',
        executable='z_plan_movegroup',
        name='z_plan_movegroup',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription([
        robot_state_publisher,
        move_group_node,
        rviz,
        z_plan_movegroup,
    ])
