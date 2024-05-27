import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    param_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(
            get_package_share_directory('minibot_arucomarker'),
            'config',
            'config.yaml')
    )

    return LaunchDescription([
        Node(
            package='minibot_arucomarker',
            executable='img_publisher',
            name='img_publisher',
            parameters=[param_dir],
            output='screen'
        ),
        Node(
            package='minibot_arucomarker',
            executable='detect_aruco3',
            name='detect_aruco3',
            parameters=[param_dir],
            output='screen'
        ),
    ])
