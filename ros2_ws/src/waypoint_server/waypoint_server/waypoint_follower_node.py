#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty
from nav2_msgs.action import NavigateToPose


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        
        # State
        self.waypoints = []
        self.current_waypoint_index = 0
        self.following = False
        
        # Action client for navigation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Subscribers
        self.waypoint_sub = self.create_subscription(
            PoseStamped,
            '/interactive_waypoints',
            self.waypoint_callback,
            queue_size=10
        )
        self.command_sub = self.create_subscription(
            Empty,
            '/waypoint_server/command',
            self.start_following_callback,
            queue_size=1
        )
        
        self.get_logger().info('Waypoint Follower node started.')

    def waypoint_callback(self, msg: PoseStamped):
        """Callback for receiving new waypoints."""
        self.get_logger().info('Received new waypoint.')
        self.waypoints.append(msg)

    def start_following_callback(self, msg: Empty):
        """Callback to start following waypoints."""
        self.get_logger().info('Starting to follow waypoints.')
        self.current_waypoint_index = 0
        self.following = True
        self.follow_next_waypoint()

    def follow_next_waypoint(self):
        """Send the next waypoint as a navigation goal."""
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info('Finished all waypoints.')
            self.waypoints = []
            self.current_waypoint_index = 0
            self.following = False
            return

        waypoint_pose = self.waypoints[self.current_waypoint_index]
        
        self.get_logger().info(
            f'Sending goal to waypoint {self.current_waypoint_index}: '
            f'({waypoint_pose.pose.position.x:.2f}, {waypoint_pose.pose.position.y:.2f})'
        )
        
        # Wait for action server
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Navigation action server not available, retrying...')
            # Schedule retry
            self.create_timer(1.0, self.follow_next_waypoint)
            return
        
        # Create goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = waypoint_pose
        
        # Send goal
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Callback for goal response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f'Goal {self.current_waypoint_index} was rejected.')
            self.current_waypoint_index += 1
            self.follow_next_waypoint()
            return

        self.get_logger().info(f'Goal {self.current_waypoint_index} accepted.')
        
        # Get result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Callback for navigation result."""
        result = future.result()
        
        if result.result.success:
            self.get_logger().info(f'Successfully reached waypoint {self.current_waypoint_index}.')
            self.current_waypoint_index += 1
            if self.following:
                self.follow_next_waypoint()
        else:
            self.get_logger().warn(
                f'Failed to reach waypoint {self.current_waypoint_index}. '
                f'Status: {result.status}'
            )
            # Try next waypoint
            self.current_waypoint_index += 1
            if self.following:
                self.follow_next_waypoint()


def main(args=None):
    rclpy.init(args=args)
#    node = WaypointFollower()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
