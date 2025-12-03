from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os
from launch_ros.actions import Node

# Modified by Claude - Added scan matcher integration and staged launch for performance
def generate_launch_description():
    launch_rplidar = LaunchConfiguration('launch_rplidar')
    launch_bno085 = LaunchConfiguration('launch_bno085')
    launch_gps = LaunchConfiguration('launch_gps')
    launch_urdf = LaunchConfiguration('launch_urdf')
    launch_nav = LaunchConfiguration('launch_nav')
    launch_scan_matcher = LaunchConfiguration('launch_scan_matcher')
    launch_waypoint_server = LaunchConfiguration('launch_waypoint_server')

    bringup_dir = FindPackageShare('robot_bringup')

    robot_description_path = PathJoinSubstitution([
        FindPackageShare('robot_bringup'),
        'urdf',
        'robot.urdf.xacro'
    ])

    waypoint_file = PathJoinSubstitution([
        FindPackageShare('waypoint_server'),
        'config',
        'waypoints.yaml'
    ])

    # Staged launch: Group 1 (Sensors) -> Group 2 (Localization/Scan Matcher) -> Group 3 (Heavy Navigation)
    # This prevents CPU spike from all systems starting simultaneously

    # Stage 1: Sensors and basic nodes (start immediately)
    stage1_sensors = [
        DeclareLaunchArgument('launch_rplidar', default_value='true'),
        DeclareLaunchArgument('launch_bno085', default_value='true'),
        DeclareLaunchArgument('launch_gps', default_value='true'),
        DeclareLaunchArgument('launch_urdf', default_value='true'),
        DeclareLaunchArgument('launch_nav', default_value='true'),
        DeclareLaunchArgument('launch_scan_matcher', default_value='true'),
        DeclareLaunchArgument('launch_waypoint_server', default_value='true'),

        # LiDAR sensor
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('sllidar_ros2'), '/launch/sllidar_s3_launch.py'
            ]),
            condition=IfCondition(launch_rplidar),
        ),
        # IMU sensor
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('bno085_driver'), '/launch/bno085_launch.py'
            ]),
            condition=IfCondition(launch_bno085)
        ),
        # GPS sensor
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                bringup_dir, '/launch/gps_launch.py'
            ]),
            condition=IfCondition(launch_gps)
        ),
        # Robot description
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
        # Motor driver (lightweight)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('roboclaw_driver'), '/launch/roboclaw_launch.py'
            ]),
            launch_arguments={
                'serial_port': '/dev/roboclaw'
            }.items()
        ),
    ]

    # Stage 2: Scan matcher (starts after 3 seconds to let sensors stabilize)
    stage2_scan_matcher = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare('scan_matcher'), '/launch/scan_matcher_launch.py'
                ]),
                condition=IfCondition(launch_scan_matcher)
            ),
        ]
    )

    # Stage 3: Heavy navigation stack (starts after 6 seconds - most computationally expensive)
    # This includes: dual EKF filters, navsat_transform, twist_mux, velocity_smoother,
    # controller_server (MPPI), planner_server, behavior_server, dual costmaps
    stage3_navigation = TimerAction(
        period=6.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare('robot_navigation'), '/launch/gps_waypoint_follower.launch.py'
                ]),
                condition=IfCondition(launch_nav)
            ),
        ]
    )

    # Stage 4: Waypoint server (starts after 8 seconds, waits for nav2)
    stage4_waypoint_server = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='waypoint_server',
                executable='gps_waypoint_handler_node',
                name='gps_waypoint_handler',
                output='screen',
                parameters=[{
                    'waypoint_file': waypoint_file,
                    'frame_id': 'map',
                    'wait_for_nav2': True,
                    'fromll_service': '/fromLL'
                }],
                condition=IfCondition(launch_waypoint_server)
            ),
        ]
    )

    return LaunchDescription(
        stage1_sensors +
        [stage2_scan_matcher, stage3_navigation, stage4_waypoint_server]
    )
