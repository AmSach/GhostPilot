from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """Launch GhostPilot core navigation stack."""

    slam_node = Node(
        package='ghostpilot_core',
        executable='slam_node',
        name='slam_node',
        output='screen',
        parameters=[{'config_file': '/config/vins_params.yaml'}],
        remappings=[
            ('/camera/image_raw', '/camera/realsense/aligned_depth_to_color/image_raw'),
            ('/imu/data', '/imu/imu_data'),
        ]
    )

    pose_bridge = Node(
        package='ghostpilot_core',
        executable='pose_bridge',
        name='pose_bridge',
        output='screen',
        parameters=[{
            'slam_pose_topic': '/ghostpilot/pose',
            'nav2_pose_topic': '/localization_pose',
        }]
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            '/opt/ros/humble/share/nav2_bringup/launch/navigation_launch.py'
        ),
        launch_arguments={
            'params_file': '/config/nav2_params.yaml',
            'use_sim_time': 'true',
        }.items()
    )

    return LaunchDescription([
        slam_node,
        pose_bridge,
        nav2_bringup,
    ])