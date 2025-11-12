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

    waypoint_interactive_markers = Node(
        package='robot_navigation',
        executable='waypoint_interactive_markers.py',
        name='waypoint_interactive_markers',
        output='screen',
        parameters=[
            {'fixed_frame': 'map'},
        ],
    )

    return LaunchDescription([
        waypoint_server,
        waypoint_follower,
        waypoint_interactive_markers,
    ])
