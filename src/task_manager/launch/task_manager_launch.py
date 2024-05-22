from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='task_manager',
            executable='order_receiver',
            name='order_receiver',
            output='screen'
        ),
        Node(
            package='task_manager',
            executable='task_allocator',
            name='task_allocator',
            output='screen'
        ),
        Node(
            package='task_manager',
            executable='robot_status_monitor',
            name='robot_status_monitor',
            output='screen'
        ),
    ])
