from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nmea_navsat_driver',
            executable='nmea_serial_driver',
            name='nmea_navsat_driver',
            parameters=[{
                'port': '/dev/gps',
                'baud': 4800,
                'frame_id': 'gps_link'
            }],
            output='screen'
        )
    ])