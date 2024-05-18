import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import LaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    param_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(
            get_package_share_directory('task_manager'),
            'param'
        )
    )
    
    rosbridge_server_dir = get_package_share_directory('rosbridge_server')

    return LaunchDescription([
        DeclareLaunchArgument(
            'param_dir',
            default_value=param_dir
        ),
        
        Node(
            package='task_manager',
            executable='task_dispather',
            name='task_dispather',
            output='screen',
            parameters=[param_dir]  # 파라미터를 사용하려면 이 부분을 추가
        ),
        IncludeLaunchDescription(
            LaunchDescriptionSource(
                os.path.join(rosbridge_server_dir, 'launch', 'rosbridge_websocket_launch.xml')
            )
        )
    ])
