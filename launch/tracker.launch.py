from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vision_tracker',
            executable='tracker_node',
            name='tracker_node',
            parameters=['config/tracker_params.yaml'],  # path is relative to this file
            output='screen'
        )
    ])
