import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
	package_share = get_package_share_directory('d2oc_algorithm')
	default_params = os.path.join(package_share, 'config', 'd2oc_params.yaml')
	
	# Path to the d2oc executable installed by ament_python
	install_dir = os.path.dirname(os.path.dirname(package_share))  # /path/to/install/d2oc_algorithm
	executable = os.path.join(install_dir, 'bin', 'd2oc')

	params_file_arg = DeclareLaunchArgument(
		'params_file',
		default_value=default_params,
		description='Path to D2OC parameter file',
	)

	# ExecuteProcess that directly runs the installed executable
	d2oc_node = ExecuteProcess(
		cmd=[executable],
		output='screen',
		shell=False,
	)

	return LaunchDescription([
		params_file_arg,
		d2oc_node,
	])
