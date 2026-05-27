import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # 获取包的路径
    pkg_path = get_package_share_directory('uc_planning_urdf')
    
    # URDF文件路径
    urdf_file = os.path.join(pkg_path, 'urdf', 'uc_planning_urdf.urdf')

    # 读取URDF文件内容并使用Command处理，确保正确发布
    robot_description = ParameterValue(
        Command(['cat ', urdf_file]),
        value_type=str
    )
    
    # 声明launch参数
    declare_model_arg = DeclareLaunchArgument(
        'model',
        default_value='',
        description='模型参数'
    )
    
    # robot_state_publisher节点 - 发布robot_description和TF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 30.0
        }]
    )
    
    # joint_state_publisher_gui节点 - 发布关节状态
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    
    # RViz2节点 - 不指定配置文件，使用默认配置
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_path, 'config', 'display.rviz')] if os.path.exists(os.path.join(pkg_path, 'config', 'display.rviz')) else []
    )
    
    return LaunchDescription([
        declare_model_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])