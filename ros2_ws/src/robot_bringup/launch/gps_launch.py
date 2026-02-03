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
                'frame_id': 'gps_link',
                # Down-weight GPS position noise in EKF by inflating reported covariance.
                # Adjust these if you have RTK or a higher-accuracy receiver.
                'override_std_dev': True,
                'lon_std_dev': 3.0,
                'lat_std_dev': 3.0,
                'alt_std_dev': 6.0,
            }],
            output='screen'
        )
    ])
