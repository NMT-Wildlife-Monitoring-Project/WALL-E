# Copyright 2018 Open Source Robotics Foundation, Inc.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
import os
import launch.actions


def generate_launch_description():
    gps_wpf_dir = get_package_share_directory(
        "robot_navigation")
    rl_params_file = os.path.join(
        gps_wpf_dir, "config", "dual_ekf_navsat_params.yaml")
    
    # Declare GPS datum arguments with safe defaults (New Mexico location)
    datum_lat = LaunchConfiguration('datum_lat')
    datum_lon = LaunchConfiguration('datum_lon')
    datum_alt = LaunchConfiguration('datum_alt')
    
    declare_datum_lat_cmd = DeclareLaunchArgument(
        'datum_lat',
        default_value='34.0658',
        description='GPS datum latitude (degrees)')
    
    declare_datum_lon_cmd = DeclareLaunchArgument(
        'datum_lon',
        default_value='-106.908',
        description='GPS datum longitude (degrees)')
    
    declare_datum_alt_cmd = DeclareLaunchArgument(
        'datum_alt',
        default_value='0.0',
        description='GPS datum altitude (meters)')

    return LaunchDescription(
        [
            declare_datum_lat_cmd,
            declare_datum_lon_cmd,
            declare_datum_alt_cmd,
            launch_ros.actions.Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_odom",
                output="screen",
                parameters=[rl_params_file],
                remappings=[("odometry/filtered", "odometry/local")],
            ),
            launch_ros.actions.Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_map",
                output="screen",
                parameters=[rl_params_file],
                remappings=[("odometry/filtered", "odometry/global")],
            ),
            launch_ros.actions.Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform",
                output="screen",
                parameters=[
                    rl_params_file,
                    {
                        "navsat_transform.datum": [datum_lat, datum_lon, datum_alt],
                    }
                ],
                remappings=[
                    ("imu/data", "imu/data"),
                    ("gps/fix", "fix"),
                    ("gps/filtered", "gps/filtered"),
                    ("odometry/gps", "odometry/gps"),
                    # Use the odom EKF output (wheel/imu/scan), not the GPS-fused map EKF,
                    # to avoid feedback loops and map frame jumps.
                    ("odometry/filtered", "odometry/local"),
                ],
            ),
        ]
    )
