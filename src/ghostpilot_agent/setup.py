#!/usr/bin/env python3
"""Setup for ghostpilot_agent ROS2 package."""

from glob import glob
from setuptools import find_packages, setup

package_name = 'ghostpilot_agent'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aman Sachan',
    maintainer_email='amansachan92905@gmail.com',
    description='Agentic AI layer: LLM mission parser + Nav2 executor',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_parser = ghostpilot_agent.mission_parser:main',
            'mission_parser_node = ghostpilot_agent.mission_parser:main',
            'executor = ghostpilot_agent.executor:main',
            'executor_node = ghostpilot_agent.executor:main',
        ],
    },
)
