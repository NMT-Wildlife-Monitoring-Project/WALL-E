#!/usr/bin/env python3
"""
SLAM Toolbox Launch File
Launches SLAM Toolbox with fake odometry for mapping
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Launch arguments
    slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('robot_navigation'),
            'config',
            'slam_params.yaml'
        ]),
        description='Full path to slam parameters file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # Get launch configurations
    slam_params_file = LaunchConfiguration('slam_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Include RPLidar launch
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_navigation'),
                'launch',
                'rplidar_launch.py'
            ])
        ])
    )

    # Fake odometry publisher
    fake_odom_node = Node(
        package='robot_navigation',
        executable='fake_odom_publisher',
        name='fake_odom_publisher',
        output='screen'
    )

    # SLAM Toolbox launch (using official online_sync_launch.py)
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'launch',
                'online_sync_launch.py'
            ])
        ]),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': use_sim_time
        }.items()
    )

    return LaunchDescription([
        slam_params_file_arg,
        use_sim_time_arg,
        rplidar_launch,
        fake_odom_node,
        slam_toolbox_launch,
    ])
