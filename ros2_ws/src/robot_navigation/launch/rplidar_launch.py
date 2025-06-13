from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for RPLidar'
    )
    
    lidar_model_arg = DeclareLaunchArgument(
        'lidar_model',
        default_value='A1M8',
        description='RPLidar model (A1M8 or S1)'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='laser_link',
        description='Frame ID for laser data'
    )

    # Get launch configurations
    serial_port = LaunchConfiguration('serial_port')
    lidar_model = LaunchConfiguration('lidar_model')
    frame_id = LaunchConfiguration('frame_id')

    # RPLidar node
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        parameters=[{
            'serial_port': serial_port,
            'serial_baudrate': 115200,  # A1M8: 115200, S1: 256000
            'frame_id': frame_id,
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard',  # A1M8: Standard, S1: DenseBoost
            'auto_standby': True,
        }],
        output='screen'
    )

    return LaunchDescription([
        serial_port_arg,
        lidar_model_arg,
        frame_id_arg,
        rplidar_node,
    ]) 