from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bno085_driver',
            executable='bno085_node',
            name='bno085_node',
            parameters=[{
                'i2c_address': 0x4B,  # Default I2C address for BNO085
                'i2c_bus': 7,  # I2C bus number on Jetson Orin Nano
                'use_interrupt': True,  # Interrupt mode ENABLED for Stage 0 (priority launch)
                'int_pin': 15,
                'int_pin_mode': 'BOARD',
                'int_edge': 'FALLING',
                'int_bouncetime_ms': 0,
                'publish_rate_hz': 50.0,
            }],
            output='screen'
        )
    ])
