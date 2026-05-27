import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_path = get_package_share_directory("uc_planning_urdf")
    urdf_file = os.path.join(pkg_path, "urdf", "uc_planning_urdf.urdf")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
        )
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=["-entity", "uc_planning_urdf", "-file", urdf_file],
    )

    return LaunchDescription([gazebo, spawn_entity])
