from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
	moveit_config = MoveItConfigsBuilder("robo_arm10", package_name="robo_arm10_moveit_config").to_moveit_configs()

	robot_state_publisher = Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		name='robot_state_publisher',
		output='screen',
		parameters=[moveit_config.robot_description],
	)

	ros2_control_node = Node(
		package="controller_manager",
		executable="ros2_control_node",
		parameters=[
			moveit_config.robot_description,
			os.path.join(
				get_package_share_directory("robo_arm10_moveit_config"),
				"config",
				"ros2_controllers.yaml",
			),
		],
		output="screen",
	)

	joint_state_broadcaster_spawner = Node(
		package="controller_manager",
		executable="spawner",
		arguments=[
			"joint_state_broadcaster",
			"--controller-manager",
			"/controller_manager",
		],
	)

	arm_controller_spawner = Node(
		package="controller_manager",
		executable="spawner",
		arguments=[
			"arm_controller",
			"--controller-manager",
			"/controller_manager",
		],
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
		arguments=['-d', rviz_config_file],
		parameters=[
			moveit_config.robot_description,
			moveit_config.robot_description_semantic,
			moveit_config.planning_pipelines,
			moveit_config.robot_description_kinematics,
		],
	)

	move_group_node = Node(
		package='moveit_ros_move_group',
		executable='move_group',
		name='move_group',
		output='screen',
		parameters=[moveit_config.to_dict()],
	)

	key_step_control_node = Node(
		package='plan',
		executable='key_step_control',
		name='key_step_control',
		output='screen',
		parameters=[
			moveit_config.robot_description,
			moveit_config.robot_description_semantic,
			moveit_config.robot_description_kinematics,
			moveit_config.planning_pipelines,
		],
	)

	return LaunchDescription([
		robot_state_publisher,
		ros2_control_node,
		joint_state_broadcaster_spawner,
		arm_controller_spawner,
		move_group_node,
		rviz,
		key_step_control_node,
	])