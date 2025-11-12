from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    waypoint_server = Node(
        package='waypoint_server',
        executable='waypoint_server_node.py',
        name='waypoint_server',
        output='screen',
    )

    waypoint_follower = Node(
        package='waypoint_server',
        executable='waypoint_follower_node.py',
        name='waypoint_follower',
        output='screen',
    )

    return LaunchDescription([
        waypoint_server,
        waypoint_follower,
    ])
