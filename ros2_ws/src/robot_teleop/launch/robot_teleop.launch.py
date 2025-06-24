#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():   
    joy_device_arg = DeclareLaunchArgument(
        'joy_device',
        default_value='/dev/input/js0',
        description='Joystick device path'
    )
    
    # Package share directories
    robot_teleop_share = FindPackageShare('robot_teleop')
    
    # Config file based on controller type
    teleop_config = PathJoinSubstitution([
        robot_teleop_share,
        'config',
        'logitech_f710.yaml'
    ])
    
    # Joy node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[
            teleop_config,
            {
                'device_name': LaunchConfiguration('joy_device'),
            }
        ],
        output='screen'
    )
    
    # Teleop twist joy node
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        parameters=[
            teleop_config,
        ],
        output='screen'
    )
    
    # Group teleop nodes
    teleop_group = GroupAction([
        joy_node,
        teleop_node
    ])
    
    return LaunchDescription([
        joy_device_arg,
        teleop_group,
    ])
