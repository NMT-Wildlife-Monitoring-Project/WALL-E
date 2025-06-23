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
    
    samples_arg = DeclareLaunchArgument(
        'samples_per_position',
        default_value='100',
        description='Number of samples to collect for each position during calibration'
    )
    
    delay_arg = DeclareLaunchArgument(
        'delay_between_samples',
        default_value='0.01',
        description='Delay between samples in seconds'
    )
    
    # Create the calibration node
    calibration_node = Node(
        package='GY88_driver',
        executable='gy88_calibration',
        name='gy88_calibration',
        parameters=[{
            'calibration_file': LaunchConfiguration('calibration_file'),
            'samples_per_position': LaunchConfiguration('samples_per_position'),
            'delay_between_samples': LaunchConfiguration('delay_between_samples')
        }],
        output='screen'
    )
    
    # Return the launch description
    return LaunchDescription([
        calibration_file_arg,
        samples_arg,
        delay_arg,
        calibration_node
    ])
