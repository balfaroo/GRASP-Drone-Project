from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='offboard',
            executable='offboard_with_depth',
            name='offboard_with_depth'
        ),
        Node(
            package='offboard',
            executable='depth_cam_node',
            name='depth_cam_node'
        )
    ])