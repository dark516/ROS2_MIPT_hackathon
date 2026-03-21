from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='simple_sim',
            executable='simple_sim_node',
            name='simple_sim',
            output='screen'
        ),
        Node(
            package='path_planner',
            executable='planner_node',
            name='path_planner',
            output='screen',
            parameters=[{'inflation_radius': 0.25, 'resolution': 0.05}]
        ),
        Node(
            package='path_follower',
            executable='follower_node',
            name='path_follower',
            output='screen'
        )
    ])
