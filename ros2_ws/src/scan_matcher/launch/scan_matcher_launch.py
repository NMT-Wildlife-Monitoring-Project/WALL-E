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
        description='Path to the rf2o odometry configuration file'
    )

    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='Topic for input laser scans'
    )

    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='false',
        description='Whether rf2o should publish odom->base_link TF'
    )

    output_odom_topic_arg = DeclareLaunchArgument(
        'output_odom_topic',
        default_value='/odom_rf2o',
        description='Topic for output rf2o odometry'
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

    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'laser_scan_topic': LaunchConfiguration('scan_topic'),
                'odom_topic': LaunchConfiguration('output_odom_topic'),
                'init_pose_from_topic': '',
                'publish_tf': LaunchConfiguration('publish_tf'),
            }
        ],
        arguments=['--ros-args', '--log-level', ['rf2o_laser_odometry:=', LaunchConfiguration('log_level')]],
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        config_file_arg,
        scan_topic_arg,
        publish_tf_arg,
        output_odom_topic_arg,
        use_sim_time_arg,
        log_level_arg,
        rf2o_node
    ])
