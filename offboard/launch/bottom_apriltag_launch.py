from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='offboard',
            executable='bottom_cam_node',
            name='bottom_cam_node'
        ),
        Node(
            package='offboard',
            executable='bottom_april',
            name='bottom_april'
        )
    ])