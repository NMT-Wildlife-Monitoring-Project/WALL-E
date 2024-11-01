from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Start the joystick node
        Node(
            package='motor_control',
            executable='motor_control_node',
            name='motor_control_node',
            output='screen',
            parameters=[{
                'cmd_vel_topic': '/cmd_vel',
                'wheel_base': 0.2,
                'wheel_diameter': 0.045,
                'max_rpm': 100,
                'min_rpm': 10,
                'motor_serial_device': '/dev/serial0',
            }]
        ),
    ])
