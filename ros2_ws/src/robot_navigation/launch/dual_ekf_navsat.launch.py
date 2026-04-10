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
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
import os


def generate_launch_description():
    gps_wpf_dir = get_package_share_directory("robot_navigation")
    rl_params_file = os.path.join(
        gps_wpf_dir, "config", "dual_ekf_navsat_params.yaml")

    use_gps = LaunchConfiguration('use_gps')

    return LaunchDescription(
        [
            DeclareLaunchArgument('use_gps', default_value='false',
                                 description='Use GPS and map-frame EKF'),

            # Always run the odom-frame EKF
            launch_ros.actions.Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_odom",
                output="screen",
                parameters=[rl_params_file],
                remappings=[("odometry/filtered", "odometry/local")],
            ),

            # With GPS: run map-frame EKF + navsat_transform
            launch_ros.actions.Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_map",
                output="screen",
                parameters=[rl_params_file],
                remappings=[("odometry/filtered", "odometry/global")],
                condition=IfCondition(use_gps),
            ),
            launch_ros.actions.Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform",
                output="screen",
                parameters=[rl_params_file],
                remappings=[
                    ("imu/data", "imu/data"),
                    ("gps/fix", "fix"),
                    ("gps/filtered", "gps/filtered"),
                    ("odometry/gps", "odometry/gps"),
                    ("odometry/filtered", "odometry/global"),
                ],
                condition=IfCondition(use_gps),
            ),

            # Without GPS: publish static identity map -> odom transform
            launch_ros.actions.Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_odom_static",
                output="screen",
                arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
                condition=UnlessCondition(use_gps),
            ),
        ]
    )
