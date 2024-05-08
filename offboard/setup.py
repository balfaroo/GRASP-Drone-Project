from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'offboard'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bryan',
    maintainer_email='balfaro@princeton.edu',
    description='Contains all scripts for GRASP drone project',
    license='BSD-3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'offboard_track_april = offboard.offboard_track_april:main',
            'bottom_april = offboard.bottom_april:main',
            'bottom_cam_node = offboard.bottom_cam_node:main',
            'bottom_cam_node_listener = offboard.bottom_cam_node_listener:main',
            'depth_cam_node = offboard.depth_cam_node:main',
            'depth_cam_listener = offboard.depth_cam_listener:main',
            'offboard_complete = offboard.offboard_complete:main',
            'new_bottom = offboard.bottom_facing_cam:main',
            'new_depth = offboard.depth_cam_lateral_rgb:main',
            'new_depth_listener = offboard.depth_listener_new:main',
            'offboard_new = offboard.offboard_alternative_bottom_facing_rgb_lateral:main',
            'pickup_from_underneath = offboard.pickup_from_underneath:main',
            'drop_object = offboard.drop_object:main'
        ],
    },
)
