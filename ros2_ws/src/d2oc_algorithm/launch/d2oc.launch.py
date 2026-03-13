import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
	package_share = get_package_share_directory('d2oc_algorithm')
	default_params = os.path.join(package_share, 'config', 'd2oc_params.yaml')

	params_file_arg = DeclareLaunchArgument(
		'params_file',
		default_value=default_params,
		description='Path to D2OC parameter file',
	)

	scan_topic_arg = DeclareLaunchArgument(
		'scan_topic',
		default_value='/scan',
		description='LaserScan topic',
	)
	odom_topic_arg = DeclareLaunchArgument(
		'odometry_topic',
		default_value='/odometry/global',
		description='Odometry topic',
	)
	costmap_topic_arg = DeclareLaunchArgument(
		'costmap_topic',
		default_value='/local_costmap/costmap',
		description='Costmap topic',
	)

	d2oc_node = Node(
		package='d2oc_algorithm',
		executable='d2oc',
		name='d2oc_explorer',
		parameters=[LaunchConfiguration('params_file')],
		remappings=[
			('/scan', LaunchConfiguration('scan_topic')),
			('/odometry/filtered', LaunchConfiguration('odometry_topic')),
			('/local_costmap/costmap', LaunchConfiguration('costmap_topic')),
		],
		output='screen',
	)

	return LaunchDescription([
		params_file_arg,
		scan_topic_arg,
		odom_topic_arg,
		costmap_topic_arg,
		d2oc_node,
	])
