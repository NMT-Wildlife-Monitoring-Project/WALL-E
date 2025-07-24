from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # use_roboclaw = LaunchConfiguration('use_roboclaw')
    use_rplidar = LaunchConfiguration('use_rplidar')
    use_bno085 = LaunchConfiguration('use_bno085')
    use_gps = LaunchConfiguration('use_gps')

    bringup_dir = FindPackageShare('robot_bringup')

    return LaunchDescription([
        # DeclareLaunchArgument('use_roboclaw', default_value='true'),
        DeclareLaunchArgument('use_rplidar', default_value='false'),
        DeclareLaunchArgument('use_bno085', default_value='false'),
        DeclareLaunchArgument('use_gps', default_value='false'),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         FindPackageShare('ros2_roboclaw_driver'), '/launch/ros2_roboclaw_driver.launch.py'
        #     ]),
        #     condition=IfCondition(use_roboclaw)
        # ),
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
            FindPackageShare('roboclaw_driver'), '/launch/roboclaw_launch.py'
            ])
        ),
    ])
