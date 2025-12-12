#!/usr/bin/env python3

import math
import os
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from robot_localization.srv import FromLL
from ament_index_python.packages import get_package_share_directory
import yaml


def yaw_to_quaternion(z_angle_rad: float):
    """
    Convert yaw-only (2D) angle into a geometry_msgs/Quaternion.
    """
    from geometry_msgs.msg import Quaternion
    half = z_angle_rad / 2.0
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


class GpsWaypointFileFollower(Node):
    """
    Node that reads GPS waypoints (lat/lon[/alt]/yaw_deg) from a YAML file,
    converts them to map-frame poses using robot_localization's FromLL service,
    and sends them to Nav2's waypoint follower via BasicNavigator.followWaypoints().

    Assumes:
    - robot_localization is running with a FromLL service (default /fromLL)
    - Nav2 and nav2_waypoint_follower are running
    - waypoint_follower has stop_on_failure:=false to auto-skip failed waypoints
    """

    def __init__(self):
        super().__init__("gps_waypoint_file_follower")

        # Get package share directory for config files
        try:
            package_share = get_package_share_directory('waypoint_server')
            default_waypoint_file = os.path.join(package_share, 'config', 'waypoints.yaml')
        except:
            # Fallback if package not found
            default_waypoint_file = ""

        # Parameters
        self.declare_parameter("waypoint_file", default_waypoint_file)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("wait_for_nav2", True)
        self.declare_parameter("fromll_service", "/fromLL")

        self.waypoint_file = self.get_parameter(
            "waypoint_file"
        ).get_parameter_value().string_value
        self.frame_id = self.get_parameter(
            "frame_id"
        ).get_parameter_value().string_value
        self.wait_for_nav2 = self.get_parameter(
            "wait_for_nav2"
        ).get_parameter_value().bool_value
        self.fromll_service_name = self.get_parameter(
            "fromll_service"
        ).get_parameter_value().string_value

        if not self.waypoint_file:
            self.get_logger().error("Parameter 'waypoint_file' is empty.")
            raise RuntimeError("waypoint_file parameter is required")

        if not os.path.exists(self.waypoint_file):
            self.get_logger().error(f"Waypoint file does not exist: {self.waypoint_file}")
            raise RuntimeError("waypoint_file not found")

        # Create FromLL service client
        self.fromll_client = self.create_client(FromLL, self.fromll_service_name)
        if not self.fromll_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                f"FromLL service '{self.fromll_service_name}' not available."
            )
            raise RuntimeError("FromLL service unavailable")

        self.get_logger().info(f"Loading GPS waypoints from: {self.waypoint_file}")
        self.poses = self._load_and_convert_waypoints(self.waypoint_file)

        if not self.poses:
            self.get_logger().error("No valid waypoints loaded; aborting.")
            raise RuntimeError("No waypoints in file")

        # BasicNavigator to talk to Nav2 (FollowWaypoints action)
        self.navigator = BasicNavigator()

        if self.wait_for_nav2:
            # Using robot_localization for GPS-based localization (no AMCL)
            self.navigator.waitUntilNav2Active(localizer="robot_localization")
            self.get_logger().info("Nav2 is active.")

        # Start once after setup
        self.started = False
        self.timer = self.create_timer(0.5, self._start_once)

    # --- internal methods ---

    def _load_and_convert_waypoints(self, filename):
        """
        1. Load GPS waypoints from YAML.
        2. For each waypoint, call FromLL to convert (lat, lon, alt) -> map (x, y, z).
        3. Build a PoseStamped in the configured frame_id with yaw_deg orientation.
        """
        with open(filename, "r") as f:
            data = yaml.safe_load(f)

        if "waypoints" not in data:
            self.get_logger().error("YAML file missing 'waypoints' key.")
            return []

        poses = []









        for i, wp in enumerate(data["waypoints"]):
            try:
                lat = float(wp["lat"])
                lon = float(wp["lon"])
                alt = float(wp.get("alt", 0.0))
                yaw_deg = float(wp.get("yaw_deg", 0.0))
            except (KeyError, ValueError, TypeError) as e:
                self.get_logger().warn(f"Skipping invalid waypoint #{i}: {wp} ({e})")
                continue

            # Call FromLL: GPS -> map
            req = FromLL.Request()
            req.ll_point.latitude = lat
            req.ll_point.longitude = lon
            req.ll_point.altitude = alt

            future = self.fromll_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)

            if future.result() is None:
                self.get_logger().warn(
                    f"FromLL failed for waypoint #{i} (lat={lat}, lon={lon}); skipping."
                )
                continue

            map_point = future.result().map_point

            pose = PoseStamped()
            pose.header.frame_id = self.frame_id
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = map_point.x
            pose.pose.position.y = map_point.y
            pose.pose.position.z = map_point.z

            yaw_rad = math.radians(yaw_deg)
            pose.pose.orientation = yaw_to_quaternion(yaw_rad)

            poses.append(pose)

            self.get_logger().info(
                f"Waypoint #{i}: (lat={lat}, lon={lon}) -> "
                f"(x={map_point.x:.2f}, y={map_point.y:.2f}), yaw={yaw_deg}°"
            )

        self.get_logger().info(f"Loaded {len(poses)} valid map-frame waypoint(s).")
        return poses

    def _start_once(self):
        """
        Send the converted waypoints to Nav2 exactly once.
        """
        if self.started:
            return
        self.started = True
        self.timer.cancel()

        self.get_logger().info("Sending waypoints to Nav2 Waypoint Follower...")
        self.navigator.followWaypoints(self.poses)

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback is not None:
                self.get_logger().info(
                    f"Currently at waypoint index: {feedback.current_waypoint}"
                )
            time.sleep(0.1)

        result = self.navigator.getResult()
        missed = getattr(result, "missed_waypoints", [])
        if missed:
            self.get_logger().warn(
                f"Navigation finished with {len(missed)} missed waypoint(s): {list(missed)}"
            )
        else:
            self.get_logger().info("Successfully reached all waypoints.")

        self.get_logger().info("GPS waypoint run complete; shutting down.")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = GpsWaypointFileFollower()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
