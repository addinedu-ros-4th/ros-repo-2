import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    robot_id = '212'  # Robot ID 

    return LaunchDescription([
        Node(
            package='task_manager',
            executable='order_receiver',
            name='order_receiver'
        ),
        Node(
            package='task_manager',
            executable='task_allocator',
            name='task_allocator'
        ),
        Node(
            package='task_manager',
            executable='robot_status_publisher',
            name='robot_status_publisher',
            parameters=[{'robot_id': robot_id}]
        )
    ])
