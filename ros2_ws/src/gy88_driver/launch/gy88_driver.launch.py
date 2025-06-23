from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    calibration_file_arg = DeclareLaunchArgument(
        'calibration_file',
        default_value='calibration.json',
        description='Path to the calibration file (absolute or relative to package config directory)'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='imu_link',
        description='Frame ID for the IMU messages'
    )
    
    update_rate_arg = DeclareLaunchArgument(
        'update_rate',
        default_value='20.0',
        description='Update rate in Hz'
    )
    
    # Get the package share directory
    pkg_share = FindPackageShare('GY88_driver')
    
    # Create the GY88 node
    gy88_node = Node(
        package='GY88_driver',
        executable='gy88_node',
        name='gy88_driver',
        parameters=[{
            'calibration_file': LaunchConfiguration('calibration_file'),
            'frame_id': LaunchConfiguration('frame_id'),
            'update_rate': LaunchConfiguration('update_rate')
        }],
        output='screen'
    )
    
    # Return the launch description
    return LaunchDescription([
        calibration_file_arg,
        frame_id_arg,
        update_rate_arg,
        gy88_node
    ])
