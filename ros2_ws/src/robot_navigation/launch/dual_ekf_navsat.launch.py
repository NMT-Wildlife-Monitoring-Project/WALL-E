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
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
import launch_ros.actions
import os


def generate_launch_description():
    gps_wpf_dir = get_package_share_directory("robot_navigation")
    rl_params_file = os.path.join(
        gps_wpf_dir, "config", "dual_ekf_navsat_params.yaml")

    use_gps = LaunchConfiguration('use_gps')
    use_slam = LaunchConfiguration('use_slam')

    return LaunchDescription(
        [
            DeclareLaunchArgument('use_gps', default_value='false',
                                 description='Use GPS and map-frame EKF'),
            DeclareLaunchArgument('use_slam', default_value='false',
                                 description='slam_toolbox owns map->odom when true'),

            # Always run the odom-frame EKF
            launch_ros.actions.Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_odom",
                output="screen",
                parameters=[rl_params_file],
                remappings=[("odometry/filtered", "odometry/local")],
            ),

            # With GPS and NOT SLAM: map EKF publishes map->odom TF.
            # SLAM always wins — when use_slam=true, slam_toolbox owns map->odom.
            launch_ros.actions.Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_map",
                output="screen",
                parameters=[rl_params_file, {'publish_tf': True}],
                remappings=[("odometry/filtered", "odometry/global")],
                condition=IfCondition(PythonExpression([
                    "'", use_gps, "' == 'true' and '", use_slam, "' != 'true'"
                ])),
            ),

            # Without GPS: no map EKF needed — nothing uses odometry/global
            # and running it risks TF conflicts with the static map->odom publisher

            # navsat_transform only needed with GPS AND not SLAM.
            # In SLAM+GPS mode, no georeference into the map frame (Phase 1 scope).
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
                condition=IfCondition(PythonExpression([
                    "'", use_gps, "' == 'true' and '", use_slam, "' != 'true'"
                ])),
            ),

            # No GPS and no SLAM: static identity map->odom so the map frame is stable
            # (dead-reckoning mode). SLAM and GPS map-EKF each take precedence if on.
            launch_ros.actions.Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_odom_static",
                output="screen",
                arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
                condition=IfCondition(PythonExpression([
                    "'", use_gps, "' != 'true' and '", use_slam, "' != 'true'"
                ])),
            ),
        ]
    )
