from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    i2c_address_arg = DeclareLaunchArgument(
        'i2c_address',
        default_value='0x4B',
        description='I2C address of the BNO085 sensor (typically 0x4A or 0x4B)'
    )
    
    i2c_bus_arg = DeclareLaunchArgument(
        'i2c_bus',
        default_value='7',
        description='I2C bus number (7 for Jetson Orin Nano, 1 for Raspberry Pi)'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='imu_link',
        description='TF frame ID for IMU (must match URDF definition in robot.urdf.xacro)'
    )
    
    enable_external_calibration_arg = DeclareLaunchArgument(
        'enable_external_calibration',
        default_value='false',
        description='Enable external calibration at startup (deprecated; use built-in calibration)'
    )
    
    return LaunchDescription([
        i2c_address_arg,
        i2c_bus_arg,
        frame_id_arg,
        enable_external_calibration_arg,
        Node(
            package='bno085_driver',
            executable='bno085_node',
            name='bno085_node',
            parameters=[{
                'i2c_address': LaunchConfiguration('i2c_address'),
                'i2c_bus': LaunchConfiguration('i2c_bus'),
                'frame_id': LaunchConfiguration('frame_id'),
                'enable_external_calibration': LaunchConfiguration('enable_external_calibration'),
            }],
            output='screen'
        )
    ])