import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
	param_dir = LaunchConfiguration( # launch파일에서 사용할 변수 정의
		'param_dir',
		default=os.path.join(
			get_package_share_directory('task_manager'),
			get_package_share_directory('rosbridge_server'),
			'param',
			)
		)

	return LaunchDescription(
		[
			DeclareLaunchArgument(
			'param_dir',
			default_value=param_dir
			),

			Node(
				package='task_manager',
				executable='task_dispather',
				name='task_dispather',
				output='screen'),

			Node(
				package='rosbridge_server',
				executable='rosbridge_websocket_launch',
				name='rosbridge_websocket_launch',
				output='screen'),
		]
	)