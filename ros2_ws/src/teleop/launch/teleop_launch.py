from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Start the joystick node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            # Adjust the joystick device if necessary
            parameters=[{
                'device_id': 0,
                'deadzone': 0.2
            }]
        ),

        # Start the teleop_twist_joy node
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            parameters=[{
                'require_enable_button': False,
                'axis_linear.x': 1,
                'scale_linear.x': 1.0,
                'axis_angular.yaw': 0,
                'scale_angular.yaw': 2.0,
                'inverted_reverse': False
            }]
        ),
    ])
