#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    joy_device_arg = DeclareLaunchArgument(
        'joy_device',
        default_value='/dev/input/js0',
        description='Joystick device path'
    )
    
    # Package share directory
    robot_teleop_share = FindPackageShare('robot_teleop')
    
    # Config file path
    config_file = PathJoinSubstitution([
        robot_teleop_share,
        'config',
        'joy_linux_f710.yaml'
    ])
    
    # Joy Linux node - better Linux joystick support
    joy_linux_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='joy_linux_node',
        parameters=[
            config_file,
            {
                'device_name': LaunchConfiguration('joy_device'),
            }
        ],
        output='screen'
    )
    
    # Teleop twist joy node - converts joystick input to cmd_vel
    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        parameters=[
            config_file,
        ],
        remappings=[('cmd_vel', '/cmd_vel_teleop')],
        output='screen'
    )

    return LaunchDescription([
        joy_device_arg,
        joy_linux_node,
        teleop_twist_joy_node
    ])
