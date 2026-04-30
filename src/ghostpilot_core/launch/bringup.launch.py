#!/usr/bin/env python3
"""
GhostPilot Core - GPS-Denied Drone Navigation Stack

Main launch file that brings up VINS-Mono SLAM, pose bridge, and Nav2 navigation.
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for GhostPilot bringup."""
    
    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )
    
    drone_name = DeclareLaunchArgument(
        'drone_name',
        default_value='iris',
        description='Name of the drone model'
    )
    
    # VINS-Mono SLAM Node
    vins_node = Node(
        package='ghostpilot_core',
        executable='slam_node',
        name='vins_slam',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'config_file': os.path.join(
                os.path.dirname(__file__), 
                '../config/vins_params.yaml'
            )
        }],
        remappings=[
            ('/camera/image', '/camera/image_raw'),
            ('/imu/data', '/imu/data'),
            ('/odometry/vis', '/vins/odometry')
        ]
    )
    
    # Pose Bridge Node - connects VINS to Nav2
    pose_bridge_node = Node(
        package='ghostpilot_core',
        executable='pose_bridge',
        name='pose_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'map_frame': 'map'
        }],
        remappings=[
            ('/vins/odometry', '/odometry/vis'),
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static')
        ]
    )
    
    # Nav2 launch (using standard Nav2 bringup)
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch/navigation_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': os.path.join(
                os.path.dirname(__file__),
                '../config/nav2_params.yaml'
            )
        }.items()
    )
    
    return LaunchDescription([
        use_sim_time,
        drone_name,
        vins_node,
        pose_bridge_node,
        nav2_bringup
    ])


# Helper for IncludeLaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory