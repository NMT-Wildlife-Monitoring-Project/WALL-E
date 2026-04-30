# Multi-Waypoint GPS Navigation Architecture & Implementation Plan

## 1. System Architecture in Detail

The multi-waypoint GPS system maps real-world coordinates onto the robot's local Cartesian map using ROS 2's `robot_localization` and `nav2` stacks. 

### The Coordinate Pipeline
1. **Input:** User defines WGS84 coordinates (Latitude, Longitude) via a YAML file, sourced from Google Maps.
2. **Translation Service (`navsat_transform_node`):** 
   - This node maintains the `datum` — the anchor point defining where `Lat, Lon (X, Y)` equals map `(0, 0)`.
   - It exposes a ROS 2 service called `/fromLL` (From Lat/Lon).
   - We pass the Lat/Lon coordinates to `/fromLL`, and it returns `x, y` positions in the `/map` frame.
3. **Action Client (`gps_waypoint_manager.py`):**
   - Accumulates the `x, y` coordinates into an array of `PoseStamped` messages.
   - Connects to the Nav2 `NavigateThroughPoses` action server.
   - Submits the array of goals.
4. **Execution (`nav2`):**
   - Nav2 computes a viable path between each generated Cartesian point.
   - It uses the local costmap (fed by RPLiDAR) to avoid dynamic/unexpected obstacles.
   - It issues velocity commands to `/cmd_vel_nav`, which `twist_mux` routes to the RoboClaw.

---

## 2. Hardware Requirements & Constraints

*   **u-blox GPS (`/dev/gps`):** Must provide a stable 3D fix `/fix` (`status.status == 0`). Accuracy drift will directly translate to pathing errors.
*   **BNO085 IMU:** Nav2 needs a valid orientation to drive. GPS does not provide yaw natively when stationary. The IMU maintains the `base_link` vector.
*   **RPLiDAR S3:** Critical for local collision monitor and local costmap to prevent the robot from blindly executing GPS vectors into trees.

---

## 3. Code & Component Implementation

Below are the exact code and configuration patterns we will integrate.

### A. The Input File: `ros2_ws/src/waypoint_server/config/route.yaml`
This file allows non-programmers to define routes simply.
```yaml
waypoints:
  - lat: 34.068130
    lon: -106.901847
  - lat: 34.068200
    lon: -106.901900
  - lat: 34.068350
    lon: -106.902100
```

### B. The Orchestrator Node: `gps_waypoint_manager.py`
This is the core Python node that does the lifting. It acts as both a Service Client to `robot_localization` and an Action Client to `Nav2`.

*Path:* `ros2_ws/src/waypoint_server/waypoint_server/gps_waypoint_manager.py`
```python
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
        self.get_logger().info(f"Loaded {len(waypoints_ll)} GPS waypoints.")

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
            
            # Synchronous call mathematically transforms coordinates based on map datum
            future = self.from_ll_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            
            if future.result() is not None:
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position = future.result().map_point
                # Default orientation (facing positive X, can be overridden if needed)
                pose.pose.orientation.w = 1.0 
                pose_goals.append(pose)
                self.get_logger().info(f"WP {index}: Converted [{wp['lat']}, {wp['lon']}] to X:{pose.pose.position.x:.2f}, Y:{pose.pose.position.y:.2f}")
            else:
                self.get_logger().error(f"Failed to convert waypoint {index}")

        # 4. Send the compiled route to Nav2
        self.get_logger().info("Connecting to Nav2 NavigateThroughPoses server...")
        self.nav_client.wait_for_server()
        
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
    rclpy.spin(manager)

if __name__ == '__main__':
    main()
```

### C. Launch File `gps_waypoint.launch.py`
We will create a launch file that explicitly runs this node while pointing it to the configuration we want to use.
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='waypoint_server',
            executable='gps_waypoint_manager',
            name='gps_waypoint_manager',
            output='screen',
            parameters=[]
        )
    ])
```

### D. System Config Modifications `dual_ekf_navsat_params.yaml`
We will make sure the EKF settings allow for the first GPS reading to define the `(0,0)` origin (datum) of the map. This makes it so we do not have to update our config file every time we transport the robot.
```yaml
navsat_transform:
  ros__parameters:
    use_odometry_yaw: true
    wait_for_datum: false
    # If wait_for_datum is false, it takes the first GPS fix as the implicit datum.
    delay: 3.0 
    publish_filtered_gps: true
    broadcast_cartesian_transform: true
```

---

## 4. Operational Workflow (End-to-End)

1. **Routing Strategy:**
   - Open Google Maps on your host PC.
   - Right-click > Copy coordinates of a start point and several subsequent physical maneuvers.
   - Paste these sequentially into `ros2_ws/src/waypoint_server/config/route.yaml`.
2. **Deploy Code:**
   - On Host: `git commit -am "added new route"` & `git push`
   - On Jetson: `git pull && cd docker && ./start_docker.sh -b`
3. **Bringup Navigation Stack:**
   - Boot inside Docker using GPS override mode:
     `./start_docker.sh -c "ros2 launch robot_bringup robot_launch.py launch_slam:=false launch_gps:=true"`
   - Ensure you see `map` → `odom` publishing via `ros2 tf2_echo map odom`.
4. **Trigger the Route Execute:**
   - Open a secondary terminal to the docker container.
   - `ros2 launch waypoint_server gps_waypoint.launch.py`
   - The robot will sequentially calculate Cartesian vectors and drive to completion.