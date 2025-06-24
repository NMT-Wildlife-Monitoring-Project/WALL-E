#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('scan_matcher'),
            'config',
            'scan_matcher_params.yaml'
        ]),
        description='Path to the scan matcher configuration file'
    )
    
    use_wheel_odom_arg = DeclareLaunchArgument(
        'use_wheel_odometry',
        default_value='true',
        description='Whether to use wheel odometry as initial guess'
    )
    
    wheel_odom_topic_arg = DeclareLaunchArgument(
        'wheel_odom_topic',
        default_value='/odom/wheels',
        description='Topic for input wheel odometry'
    )
    
    output_odom_topic_arg = DeclareLaunchArgument(
        'output_odom_topic', 
        default_value='/odom/matched',
        description='Topic for output scan-matched odometry'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='INFO',
        description='Log level (DEBUG, INFO, WARN, ERROR)',
        choices=['DEBUG', 'INFO', 'WARN', 'ERROR']
    )
    
    # Scan matcher node
    scan_matcher_node = Node(
        package='scan_matcher',
        executable='scan_matcher_node',
        name='scan_matcher_node',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'use_wheel_odometry': LaunchConfiguration('use_wheel_odometry'),
                'wheel_odom_topic': LaunchConfiguration('wheel_odom_topic'),
                'output_odom_topic': LaunchConfiguration('output_odom_topic'),
            }
        ],
        arguments=['--ros-args', '--log-level', ['scan_matcher_node:=', LaunchConfiguration('log_level')]],
        output='screen',
        emulate_tty=True
    )
    
    return LaunchDescription([
        config_file_arg,
        use_wheel_odom_arg,
        wheel_odom_topic_arg,
        output_odom_topic_arg,
        use_sim_time_arg,
        log_level_arg,
        scan_matcher_node
    ])
