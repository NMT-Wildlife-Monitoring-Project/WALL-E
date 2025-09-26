from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare all arguments
    args = [
        DeclareLaunchArgument('serial_port', default_value='/dev/roboclaw'),
        DeclareLaunchArgument('baudrate', default_value='9600'),
        DeclareLaunchArgument('address', default_value='128'),
        DeclareLaunchArgument('qppr', default_value='6533'),
        DeclareLaunchArgument('accel', default_value='1.5'),
        DeclareLaunchArgument('max_speed', default_value='1.0'),
        DeclareLaunchArgument('max_speed_qpps', default_value='10560'),
        DeclareLaunchArgument('accel_qpps', default_value='-1'),
        DeclareLaunchArgument('wheel_separation', default_value='0.40'),
        DeclareLaunchArgument('wheel_diameter', default_value='0.095'),
        DeclareLaunchArgument('m1_reverse', default_value='True'),
        DeclareLaunchArgument('m2_reverse', default_value='False'),
        DeclareLaunchArgument('odom_publish_rate', default_value='50'),
        DeclareLaunchArgument('status_publish_rate', default_value='5'),
        DeclareLaunchArgument('status_topic', default_value='roboclaw_status'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='cmd_vel'),
        DeclareLaunchArgument('odom_topic', default_value='odom'),
        DeclareLaunchArgument('odom_frame_id', default_value='odom'),
        DeclareLaunchArgument('base_frame_id', default_value='base_link'),
        DeclareLaunchArgument('cmd_vel_timeout', default_value='1.0'),
    ]

    # Pass all arguments as parameters to the node
    node_params = [
        {'serial_port': LaunchConfiguration('serial_port')},
        {'baudrate': LaunchConfiguration('baudrate')},
        {'address': LaunchConfiguration('address')},
        {'qppr': LaunchConfiguration('qppr')},
        {'accel': LaunchConfiguration('accel')},
        {'max_speed': LaunchConfiguration('max_speed')},
        {'max_speed_qpps': LaunchConfiguration('max_speed_qpps')},
        {'accel_qpps': LaunchConfiguration('accel_qpps')},
        {'wheel_separation': LaunchConfiguration('wheel_separation')},
        {'wheel_diameter': LaunchConfiguration('wheel_diameter')},
        {'m1_reverse': LaunchConfiguration('m1_reverse')},
        {'m2_reverse': LaunchConfiguration('m2_reverse')},
        {'odom_publish_rate': LaunchConfiguration('odom_publish_rate')},
        {'status_publish_rate': LaunchConfiguration('status_publish_rate')},
        {'status_topic': LaunchConfiguration('status_topic')},
        {'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic')},
        {'odom_topic': LaunchConfiguration('odom_topic')},
        {'odom_frame_id': LaunchConfiguration('odom_frame_id')},
        {'base_frame_id': LaunchConfiguration('base_frame_id')},
        {'cmd_vel_timeout': LaunchConfiguration('cmd_vel_timeout')},
    ]

    return LaunchDescription(
        args + [
            Node(
                package='roboclaw_driver',
                executable='roboclaw_node',
                name='roboclaw_node',
                output='screen',
                parameters=node_params,
            )
        ]
    )
