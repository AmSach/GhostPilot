from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Launch GhostPilot core navigation stack with proper path resolution."""

    # Get package share directory (FIXED: no hardcoded paths)
    pkg_share = get_package_share_directory('ghostpilot_core')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
        description='Full path to Nav2 params file'
    )

    slam_node = Node(
        package='ghostpilot_core',
        executable='slam_node',
        name='slam_node',
        output='screen',
        parameters=[{
            'config_file': os.path.join(pkg_share, 'config', 'vins_params.yaml'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
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
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    # Nav2 bringup - use package share directory
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            )
        ),
        launch_arguments={
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )

    return LaunchDescription([
        use_sim_time_arg,
        config_file_arg,
        slam_node,
        pose_bridge,
        nav2_bringup,
    ])