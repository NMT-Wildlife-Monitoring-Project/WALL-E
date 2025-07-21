from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bno085_driver',
            executable='bno085_node',
            name='bno085_node',
            parameters=[{
                'i2c_address': 0x4B  # Default I2C address for BNO085
            }],
            output='screen'
        )
    ])