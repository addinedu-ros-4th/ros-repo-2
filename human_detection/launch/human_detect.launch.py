from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('human_detection'),
        'params',
        'model.yaml'
    )

    return LaunchDescription([
        Node(
            package='human_detection',
            executable='human_detect',
            name='human_detect',
            parameters=[params]
        ),
    ])