from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os
from launch_ros.actions import Node

def generate_launch_description():
    # use_roboclaw = LaunchConfiguration('use_roboclaw')
    use_rplidar = LaunchConfiguration('use_rplidar')
    use_bno085 = LaunchConfiguration('use_bno085')
    use_gps = LaunchConfiguration('use_gps')

    bringup_dir = FindPackageShare('robot_bringup')

    robot_description_path = os.path.join(
        FindPackageShare('robot_bringup'),
        'urdf',
        'robot.urdf.xacro'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rplidar', default_value='true'),
        DeclareLaunchArgument('use_bno085', default_value='true'),
        DeclareLaunchArgument('use_gps', default_value='true'),
        DeclareLaunchArgument('use_urdf', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('sllidar_ros2'), '/launch/sllidar_s3_launch.py'
            ]),
            condition=IfCondition(use_rplidar)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('bno085_driver'), '/launch/bno085_launch.py'
            ]),
            condition=IfCondition(use_bno085)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                bringup_dir, '/launch/gps_launch.py'
            ]),
            condition=IfCondition(use_gps)
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', robot_description_path])
            }],
            condition=IfCondition(LaunchConfiguration('use_urdf'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('roboclaw_driver'), '/launch/roboclaw_launch.py'
            ])
        ),
    ])
