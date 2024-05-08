'''launch file for starting depth cam, bottom cam, and offboard control nodes'''
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='offboard',
            executable='offboard_new',
            name='offboard_new'
        ),
        Node(
            package='offboard',
            executable='new_depth',
            name='new_depth'
        ),
        Node(
            package='offboard',
            executable='new_bottom',
            name='new_bottom'
        )
    ])