from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tello',
            executable='controller_node',
            output='screen'
        ),
        Node(
            package='tello',
            executable='state_node',
            output='screen'
        ),
        Node(
            package='tello',
            executable='camera_node',
            output='screen'
        ),
    ])
