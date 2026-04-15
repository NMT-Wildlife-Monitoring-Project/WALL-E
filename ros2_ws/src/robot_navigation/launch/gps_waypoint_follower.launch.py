# Copyright (c) 2018 Intel Corporation
#
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

import os
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import (
    AndSubstitution,
    LaunchConfiguration,
    NotSubstitution,
)
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    package_dir = get_package_share_directory(
        "robot_navigation")
    launch_dir = os.path.join(package_dir, 'launch')
    params_dir = os.path.join(package_dir, "config")
    nav2_params = os.path.join(params_dir, "nav2_no_map_params.yaml")
    rviz_config = os.path.join(params_dir, 'walle_default.rviz')
    configured_params = RewrittenYaml(
        source_file=nav2_params, root_key="", param_rewrites="", convert_types=True
    )

    use_rviz = LaunchConfiguration('use_rviz')
    use_mapviz = LaunchConfiguration('use_mapviz')
    launch_waypoint_follower = LaunchConfiguration('launch_waypoint_follower')
    use_gps = LaunchConfiguration('use_gps')
    launch_slam = LaunchConfiguration('launch_slam')
    slam_params_file = os.path.join(params_dir, 'slam_toolbox_params.yaml')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='False',
        description='Whether to start RVIZ')

    declare_use_mapviz_cmd = DeclareLaunchArgument(
        'use_mapviz',
        default_value='False',
        description='Whether to start mapviz')
    declare_launch_waypoint_follower_cmd = DeclareLaunchArgument(
        'launch_waypoint_follower',
        default_value='False',
        description='Whether to auto-start GPS waypoint following')

    declare_use_gps_cmd = DeclareLaunchArgument(
        'use_gps',
        default_value='false',
        description='Use GPS and map-frame EKF')

    declare_launch_slam_cmd = DeclareLaunchArgument(
        'launch_slam',
        default_value='false',
        description='Launch slam_toolbox for online 2D SLAM. '
                    'Mutually exclusive with use_gps.')

    # Fail fast if both GPS and SLAM are enabled — both would fight to publish
    # map->odom, causing TF conflicts and unpredictable localization.
    def _validate_localization_mode(context):
        gps = context.perform_substitution(use_gps).lower() in ('true', '1')
        slam = context.perform_substitution(launch_slam).lower() in ('true', '1')
        if gps and slam:
            raise RuntimeError(
                'launch_slam and use_gps cannot both be true: '
                'they would both publish map->odom. Pick one.'
            )
        return []

    validate_mode_cmd = OpaqueFunction(function=_validate_localization_mode)

    # Static map->odom identity is only needed when neither GPS nor SLAM owns it.
    publish_static_map_tf = AndSubstitution(
        NotSubstitution(use_gps),
        NotSubstitution(launch_slam),
    )

    robot_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'dual_ekf_navsat.launch.py')),
        launch_arguments={
            'use_gps': use_gps,
            'publish_map_to_odom_static': publish_static_map_tf,
        }.items()
    )

    slam_toolbox_cmd = launch_ros.actions.Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file],
        condition=IfCondition(launch_slam),
    )

    # Add twist_mux node before navigation to arbitrate teleop vs nav
    twist_mux_yaml = os.path.join(params_dir, 'twist_mux.yaml')
    twist_mux_cmd = launch_ros.actions.Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_yaml]
    )

    # GPS waypoint handler node
    gps_waypoint_handler_cmd = launch_ros.actions.Node(
        package='waypoint_server',
        executable='gps_waypoint_handler_node',
        name='gps_waypoint_handler',
        output='screen',
        parameters=[{
            'waypoint_file': os.path.join(get_package_share_directory('waypoint_server'), 'config', 'waypoints.yaml'),
            'frame_id': 'map',
            'wait_for_nav2': True,
            'fromll_service': '/fromLL'
        }],
        condition=IfCondition(launch_waypoint_follower)
    )

    navigation2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "params_file": configured_params,
            "autostart": "True",
        }.items(),
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", 'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={
            'rviz_config': rviz_config,
        }.items()
    )

    mapviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'mapviz.launch.py')),
        condition=IfCondition(use_mapviz)
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Validate that localization modes aren't conflicting
    ld.add_action(validate_mode_cmd)

    # robot localization launch
    ld.add_action(robot_localization_cmd)

    # slam_toolbox (conditional)
    ld.add_action(slam_toolbox_cmd)

    # twist_mux
    ld.add_action(twist_mux_cmd)

    # GPS waypoint handler
    ld.add_action(gps_waypoint_handler_cmd)

    # navigation2 launch
    ld.add_action(navigation2_cmd)

    # viz launch
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(declare_use_mapviz_cmd)
    ld.add_action(mapviz_cmd)
    ld.add_action(declare_launch_waypoint_follower_cmd)
    ld.add_action(declare_use_gps_cmd)
    ld.add_action(declare_launch_slam_cmd)

    return ld
