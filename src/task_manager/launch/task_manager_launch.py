from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='task_manager',
            executable='robot_controller',
            name='robot_controller'
        ),
        Node(
            package='task_manager',
            executable='task_dispatcher',
            name='task_dispatcher'
        ),
    ])
