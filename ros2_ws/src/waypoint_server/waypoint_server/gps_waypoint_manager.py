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
        self.nav_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self.from_ll_cli = self.create_client(FromLL, '/fromLL')
        self.get_logger().info("Initializing GPS Waypoint Manager...")

    def load_waypoints(self):
        yaml_path = os.path.join(
            get_package_share_directory('waypoint_server'),
            'config',
            'route.yaml'
        )
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)
        return data.get('waypoints', [])

def main(args=None):
    rclpy.init(args=args)
    manager = GpsWaypointManager()
    
    # 1. Load the YAML file
    waypoints_ll = manager.load_waypoints()
    manager.get_logger().info(f"Loaded {len(waypoints_ll)} GPS waypoints.")

    if not waypoints_ll:
        manager.get_logger().error("No waypoints found in YAML file!")
        manager.destroy_node()
        rclpy.shutdown()
        return

    # 2. Wait for the fromLL service (Ensures navsat_transform is ready)
    while not manager.from_ll_cli.wait_for_service(timeout_sec=1.0):
        manager.get_logger().warn('/fromLL service not available, waiting...')

    # 3. Convert all Lat/Lon to Map Poses
    pose_goals = []
    for index, wp in enumerate(waypoints_ll):
        req = FromLL.Request()
        req.ll_point.latitude = wp['lat']
        req.ll_point.longitude = wp['lon']
        req.ll_point.altitude = 0.0
        
        # Spin until the service call is complete
        future = manager.from_ll_cli.call_async(req)
        rclpy.spin_until_future_complete(manager, future)
        
        if future.result() is not None:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = manager.get_clock().now().to_msg()
            pose.pose.position = future.result().map_point
            # Default orientation
            pose.pose.orientation.w = 1.0 
            pose_goals.append(pose)
            manager.get_logger().info(f"WP {index+1}: Converted [{wp['lat']}, {wp['lon']}] to X:{pose.pose.position.x:.2f}, Y:{pose.pose.position.y:.2f}")
        else:
            manager.get_logger().error(f"Failed to convert waypoint {index+1}")

    # 4. Send the compiled route to Nav2
    manager.get_logger().info("Connecting to Nav2 NavigateThroughPoses server...")
    if not manager.nav_client.wait_for_server(timeout_sec=5.0):
        manager.get_logger().error("Action server not available! Ensure Nav2 is running.")
        manager.destroy_node()
        rclpy.shutdown()
        return
    
    goal_msg = NavigateThroughPoses.Goal()
    goal_msg.poses = pose_goals
    
    manager.get_logger().info(f"Dispatching route with {len(pose_goals)} poses to Nav2.")
    send_goal_future = manager.nav_client.send_goal_async(goal_msg)
    
    # Spin until Nav2 accepts or rejects the goal
    rclpy.spin_until_future_complete(manager, send_goal_future)
    
    goal_handle = send_goal_future.result()
    if not goal_handle.accepted:
        manager.get_logger().error('Route rejected by Nav2.')
        manager.destroy_node()
        rclpy.shutdown()
        return

    manager.get_logger().info('Route accepted by Nav2, executing path! Please monitor.')
    get_result_future = goal_handle.get_result_async()
    
    # Spin until Nav2 finishes driving the route
    rclpy.spin_until_future_complete(manager, get_result_future)
    
    manager.get_logger().info('Nav2 routing operation completed.')
    
    manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()