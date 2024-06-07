import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    param_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(
            get_package_share_directory('human_following'),
            'config',
            'config.yaml')
    )

    return LaunchDescription([
        Node(
            package='human_following',
            executable='following',
            name='following',
            parameters=[param_dir],
            output='screen'
        )
    ])
