import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='waypoint_server',
            executable='gps_waypoint_manager',
            name='gps_waypoint_manager',
            output='screen',
            parameters=[]
        )
    ])