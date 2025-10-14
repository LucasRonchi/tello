from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tello',
            executable='control_node',
            name='tello_control_node',
            output='screen'
        ),
        Node(
            package='tello',
            executable='state_node',
            name='tello_state_node',
            output='screen'
        ),
        Node(
            package='tello',
            executable='camera_node',
            name='tello_camera_node',
            output='screen'
        ),
    ])
