# Launch slam_toolbox in async online mapping mode.
# Caller is responsible for gating this include (we never
# start slam_toolbox unconditionally).
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('robot_navigation'),
        'config',
        'slam_toolbox_params.yaml',
    )

    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[params_file],
        ),
    ])
