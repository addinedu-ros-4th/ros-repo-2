import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    param_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(
            get_package_share_directory('robot_aruco'),
            'config',
            'config.yaml')
    )

    return LaunchDescription([
        Node(
            package='robot_aruco',
            executable='robot_cam',
            name='robot_cam',
            parameters=[param_dir],
            output='screen'
        ),
        Node(
            package='robot_aruco',
            executable='aruco_detection',
            name='aruco_detection',
            parameters=[param_dir],
            output='screen'
        ),
        Node(
            package='robot_aruco',
            executable='step_control',
            name='step_control',
            parameters=[param_dir],
            output='screen'
        ),
        Node(
            package='robot_aruco',
            executable='button_lcd_control',
            name='button_lcd_control',
            parameters=[param_dir],
            output='screen'
        ),
    ])
