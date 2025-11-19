from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os
from launch_ros.actions import Node

def generate_launch_description():
    launch_rplidar = LaunchConfiguration('launch_rplidar')
    launch_bno085 = LaunchConfiguration('launch_bno085')
    launch_gps = LaunchConfiguration('launch_gps')
    launch_urdf = LaunchConfiguration('launch_urdf')
    launch_nav = LaunchConfiguration('launch_nav')

    bringup_dir = FindPackageShare('robot_bringup')

    robot_description_path = PathJoinSubstitution([
        FindPackageShare('robot_bringup'),
        'urdf',
        'robot.urdf.xacro'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('launch_rplidar', default_value='true'),
        DeclareLaunchArgument('launch_bno085', default_value='true'),
        DeclareLaunchArgument('launch_gps', default_value='true'),
        DeclareLaunchArgument('launch_urdf', default_value='true'),
        DeclareLaunchArgument('launch_nav', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('sllidar_ros2'), '/launch/sllidar_s3_launch.py'
            ]),
            condition=IfCondition(launch_rplidar),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('bno085_driver'), '/launch/bno085_launch.py'
            ]),
            condition=IfCondition(launch_bno085)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                bringup_dir, '/launch/gps_launch.py'
            ]),
            condition=IfCondition(launch_gps)
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', robot_description_path])
            }],
            condition=IfCondition(LaunchConfiguration('launch_urdf'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('roboclaw_driver'), '/launch/roboclaw_launch.py'
            ]),
            launch_arguments={
                'serial_port': '/dev/roboclaw'
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('robot_navigation'), '/launch/gps_waypoint_follower.launch.py'
            ]),
            condition=IfCondition(launch_nav)
        )
    ])
