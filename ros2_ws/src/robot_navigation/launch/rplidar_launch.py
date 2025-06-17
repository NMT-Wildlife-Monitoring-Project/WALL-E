#!/usr/bin/env python3
"""
RPLidar Launch File
Launches the RPLidar A1 sensor
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Launch arguments
    device_port_arg = DeclareLaunchArgument(
        'device_port',
        default_value='/dev/ttyUSB0',
        description='RPLidar device port'
    )
    
    scan_mode_arg = DeclareLaunchArgument(
        'scan_mode',
        default_value='Standard',
        description='RPLidar scan mode'
    )
    
    # Get launch configurations
    device_port = LaunchConfiguration('device_port')
    scan_mode = LaunchConfiguration('scan_mode')

    # Include the sllidar_a1_launch.py from sllidar_ros2 package
    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('sllidar_ros2'),
                'launch',
                'sllidar_a1_launch.py'
            )
        ]),
        launch_arguments={
            'serial_port': device_port,
            'scan_mode': scan_mode,
            'frame_id': 'laser'
        }.items()
    )

    # Static transform from base_link to laser
    laser_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser'],
        output='screen'
    )

    return LaunchDescription([
        device_port_arg,
        scan_mode_arg,
        sllidar_launch,
        laser_tf_node,
    ])
