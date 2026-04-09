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
    launch_d2oc = LaunchConfiguration('launch_d2oc')
    use_rviz = LaunchConfiguration('use_rviz')
    launch_waypoint_follower = LaunchConfiguration('launch_waypoint_follower')
    rf2o_scan_topic = LaunchConfiguration('rf2o_scan_topic')
    rf2o_odom_topic = LaunchConfiguration('rf2o_odom_topic')
    d2oc_scan_topic = LaunchConfiguration('d2oc_scan_topic')
    d2oc_odom_topic = LaunchConfiguration('d2oc_odom_topic')
    d2oc_costmap_topic = LaunchConfiguration('d2oc_costmap_topic')

    bringup_dir = FindPackageShare('robot_bringup')

    robot_description_path = PathJoinSubstitution([
        FindPackageShare('robot_bringup'),
        'urdf',
        'robot.urdf.xacro'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('launch_rplidar', default_value='true'),
        DeclareLaunchArgument('launch_bno085', default_value='true'),
        DeclareLaunchArgument('launch_gps', default_value='false'),
        DeclareLaunchArgument('launch_urdf', default_value='true'),
        DeclareLaunchArgument('launch_nav', default_value='true'),
        DeclareLaunchArgument('launch_d2oc', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('launch_waypoint_follower', default_value='false'),
        DeclareLaunchArgument('rf2o_scan_topic', default_value='/scan'),
        DeclareLaunchArgument('rf2o_odom_topic', default_value='odom_rf2o'),
        DeclareLaunchArgument('d2oc_scan_topic', default_value='/scan'),
        DeclareLaunchArgument('d2oc_odom_topic', default_value='/odometry/global'),
        DeclareLaunchArgument('d2oc_costmap_topic', default_value='/local_costmap/costmap'),

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
            condition=IfCondition(launch_bno085),
            launch_arguments={
                'frame_id': 'imu_link',
            }.items()
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
            condition=IfCondition(launch_nav),
            launch_arguments={
                'use_rviz': use_rviz,
                'launch_waypoint_follower': launch_waypoint_follower,
            }.items()
        ),
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',
            parameters=[{
                'laser_scan_topic': rf2o_scan_topic,
                'odom_topic': rf2o_odom_topic,
                'init_pose_from_topic': '',
                'publish_tf': False,
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'freq': 20.0,
            }],
            condition=IfCondition(launch_nav),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('d2oc_algorithm'), '/launch/d2oc.launch.py'
            ]),
            condition=IfCondition(launch_d2oc),
            launch_arguments={
                'scan_topic': d2oc_scan_topic,
                'odometry_topic': d2oc_odom_topic,
                'costmap_topic': d2oc_costmap_topic,
            }.items()
        )
    ])
