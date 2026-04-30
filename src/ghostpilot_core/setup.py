#!/usr/bin/env python3
"""Setup for ghostpilot_core ROS2 package."""

from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'ghostpilot_core'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        # Include config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aman Sachan',
    maintainer_email='amansachan92905@gmail.com',
    description='Core navigation stack: SLAM + Nav2 bridge for GPS-denied drone flight',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'slam_node = ghostpilot_core.slam_node:main',
            'pose_bridge = ghostpilot_core.pose_bridge:main',
        ],
    },
)