#!/usr/bin/env python3
"""
SLAM with Visualization Launch File
Launches SLAM Toolbox with RViz for visualization
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Launch arguments
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('robot_navigation'),
            'config',
            'slam_rviz.rviz'
        ]),
        description='Full path to rviz config file'
    )

    # Get launch configurations
    rviz_config = LaunchConfiguration('rviz_config')

    # Include SLAM launch
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_navigation'),
                'launch',
                'slam_launch.py'
            ])
        ])
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        rviz_config_arg,
        slam_launch,
        rviz_node,
    ])
