from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directories
    robo_arm_desc_pkg = get_package_share_directory('robo_arm_description')
    urdf_file = os.path.join(robo_arm_desc_pkg, 'urdf', 'robo_arm.urdf')
    
    # 使用 MoveItConfigsBuilder 正确配置所有参数
    moveit_config = (
        MoveItConfigsBuilder("robotic_arm", package_name="robo_arm_moveit_config")
        .robot_description(file_path=urdf_file)
        .robot_description_semantic(file_path="config/robo_arm.srdf")
        .planning_pipelines(pipelines=["ompl"])
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
    
    # Static TF: world -> base_link
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0.7', '0', '0.2', '0', '0', '0', 'world', 'base_link'],
        output='screen',
    )
    
    # Joint State Publisher GUI
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )

    # RViz
    try:
        rviz_config = os.path.join(
            get_package_share_directory('robo_arm_description'), 
            'rviz', 'display.rviz'
        )
    except:
        rviz_config = None
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config] if rviz_config and os.path.exists(rviz_config) else [],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
        ],
    )

    # MoveGroup 节点 - 使用 MoveItConfigsBuilder 生成的完整配置
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.trajectory_execution,
        ],
    )

    # Z Plan节点
    z_plan_node = Node(
        package='z_plan',
        executable='z_plan_node',
        name='z_plan_node',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
        ],
    )

    return LaunchDescription([
        robot_state_publisher,
        static_tf,
        joint_state_publisher_gui,
        move_group_node,
        rviz,
        z_plan_node,
    ])
