# Launch slam_toolbox in async online mapping mode.
# Caller is responsible for gating this include (we never
# start slam_toolbox unconditionally).
#
# async_slam_toolbox_node is a lifecycle node. It does NOT self-activate;
# it must be walked through CONFIGURE -> ACTIVATE externally. We mirror
# upstream's online_async_launch.py pattern: emit CONFIGURE on start, then
# an event handler emits ACTIVATE once CONFIGURE succeeds. Without these,
# the node sits in 'unconfigured' forever (no /scan subscription, no
# map->odom TF), which breaks nav2 planner_server activation.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, LogInfo, RegisterEventHandler
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('robot_navigation'),
        'config',
        'slam_toolbox_params.yaml',
    )

    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[params_file],
    )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg='[LifecycleLaunch] slam_toolbox is activating.'),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_node),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    return LaunchDescription([
        slam_node,
        configure_event,
        activate_event,
    ])
