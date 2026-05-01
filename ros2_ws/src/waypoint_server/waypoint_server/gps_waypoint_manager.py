import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import yaml
import os

from nav2_msgs.action import NavigateThroughPoses
from robot_localization.srv import FromLL
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory

class GpsWaypointManager(Node):
    def __init__(self):
        super().__init__('gps_waypoint_manager')
        
        # Action client for Nav2
        self.nav_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        
        # Service client to convert Lat/Lon to Map X/Y
        self.from_ll_cli = self.create_client(FromLL, '/fromLL')
        
        self.get_logger().info("Initializing GPS Waypoint Manager...")
        # Need to run this slightly deferred to allow the node to fully initialize
        self.timer = self.create_timer(1.0, self.run_mission_timer_callback)

    def run_mission_timer_callback(self):
        self.timer.cancel() # Only run once
        self.run_mission()

    def run_mission(self):
        # 1. Load the YAML file
        yaml_path = os.path.join(
            get_package_share_directory('waypoint_server'),
            'config',
            'route.yaml'
        )
        
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)
            
        waypoints_ll = data.get('waypoints', [])
        self.get_logger().info(f"Loaded {len(waypoints_ll)} GPS waypoints from {yaml_path}")

        if not waypoints_ll:
            self.get_logger().error("No waypoints found in YAML file!")
            return

        # 2. Wait for the fromLL service (Ensures navsat_transform is ready)
        while not self.from_ll_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('/fromLL service not available, waiting...')

        # 3. Convert all Lat/Lon to Map Poses
        pose_goals = []
        for index, wp in enumerate(waypoints_ll):
            req = FromLL.Request()
            req.ll_point.latitude = wp['lat']
            req.ll_point.longitude = wp['lon']
            req.ll_point.altitude = 0.0
            
            # Send sync request (spins locally so it doesn't block the whole executor)
            future = self.from_ll_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            
            if future.result() is not None:
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position = future.result().map_point
                # Default orientation
                pose.pose.orientation.w = 1.0 
                pose_goals.append(pose)
                self.get_logger().info(f"WP {index+1}: Converted [{wp['lat']}, {wp['lon']}] to X:{pose.pose.position.x:.2f}, Y:{pose.pose.position.y:.2f}")
            else:
                self.get_logger().error(f"Failed to convert waypoint {index}")

        # 4. Send the compiled route to Nav2
        self.get_logger().info("Connecting to Nav2 NavigateThroughPoses server...")
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available! Ensure Nav2 is running.")
            return
        
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = pose_goals
        
        self.get_logger().info(f"Dispatching route with {len(pose_goals)} poses to Nav2.")
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Route rejected by Nav2.')
            return

        self.get_logger().info('Route accepted by Nav2, executing path!')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Nav2 routing operation completed.')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    manager = GpsWaypointManager()
    try:
        rclpy.spin(manager)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()