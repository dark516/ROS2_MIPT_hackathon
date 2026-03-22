import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_ws',
            executable='visualizer',
            name='ros2_visualizer',
            output='screen',
            parameters=[
                {'use_sim_time': False},
                {'log_level': 'info'}
            ]
        )
    ])