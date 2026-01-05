from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 获取包路径
    z_plan_pkg = get_package_share_directory('z_plan')
    robo_arm_desc_pkg = get_package_share_directory('robo_arm_description')
    
    # 读取URDF内容
    with open(os.path.join(robo_arm_desc_pkg, 'urdf', 'robo_arm.urdf'), 'r') as f:
        robot_description = f.read()
    
    # 读取SRDF内容
    try:
        moveit_config_pkg = get_package_share_directory('robo_arm_moveit_config')
        with open(os.path.join(moveit_config_pkg, 'config', 'robo_arm.srdf'), 'r') as f:
            robot_description_semantic = f.read()
    except:
        robot_description_semantic = ""
    
    # 读取运动学配置
    try:
        with open(os.path.join(moveit_config_pkg, 'config', 'kinematics.yaml'), 'r') as f:
            kinematics_yaml = f.read()
    except:
        kinematics_yaml = ""
    
    # 读取OMPL规划配置
    try:
        with open(os.path.join(moveit_config_pkg, 'config', 'ompl_planning.yaml'), 'r') as f:
            ompl_planning_yaml = f.read()
    except:
        ompl_planning_yaml = ""
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )
    
    # Static TF: world -> base_link (根据URDF中的固定关节)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0.7', '0', '0.2', '0', '0', '0', 'world', 'base_link'],
        output='screen',
    )
    
    # Joint State Publisher GUI（用于手动调整机械臂）
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )

    # RViz - 优先使用robo_arm_description的配置
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
            {
                'robot_description': robot_description,
                'robot_description_semantic': robot_description_semantic,
            }
        ],
    )

    # Z Plan节点
    z_plan_node = Node(
        package='z_plan',
        executable='z_plan_node',
        name='z_plan_node',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'robot_description_semantic': robot_description_semantic,
                'robot_description_kinematics': kinematics_yaml,
            }
        ],
    )

    return LaunchDescription([
        robot_state_publisher,
        static_tf,
        joint_state_publisher_gui,
        rviz,
        z_plan_node,
    ])
