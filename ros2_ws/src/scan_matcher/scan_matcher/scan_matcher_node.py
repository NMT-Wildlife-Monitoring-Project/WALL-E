import rclpy
from rclpy.node import Node
import numpy as np
from math import cos, sin, atan2
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseWithCovariance, Twist, Quaternion
from builtin_interfaces.msg import Time
from scipy.spatial import KDTree


class ScanMatcherNode(Node):
    def __init__(self):
        super().__init__('scan_matcher_node')

        # Declare parameters
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('wheel_odom_topic', '/odom')
        self.declare_parameter('output_odom_topic', '/odom_matched')
        self.declare_parameter('use_wheel_odometry', True)
        self.declare_parameter('max_iterations', 20)
        self.declare_parameter('tolerance', 1e-4)
        self.declare_parameter('max_correspondence_distance', 0.5)
        self.declare_parameter('min_scan_range', 0.1)
        self.declare_parameter('max_scan_range', 10.0)
        self.declare_parameter('downsample_factor', 2)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')

        # Get parameters
        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.wheel_odom_topic = self.get_parameter('wheel_odom_topic').get_parameter_value().string_value
        self.output_odom_topic = self.get_parameter('output_odom_topic').get_parameter_value().string_value
        self.use_odometry = self.get_parameter('use_wheel_odometry').get_parameter_value().bool_value
        self.max_iterations = self.get_parameter('max_iterations').get_parameter_value().integer_value
        self.tolerance = self.get_parameter('tolerance').get_parameter_value().double_value
        self.max_corr_dist = self.get_parameter('max_correspondence_distance').get_parameter_value().double_value
        self.min_range = self.get_parameter('min_scan_range').get_parameter_value().double_value
        self.max_range = self.get_parameter('max_scan_range').get_parameter_value().double_value
        self.downsample_factor = self.get_parameter('downsample_factor').get_parameter_value().integer_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, 10)
        
        # Optional wheel odometry subscription
        self.wheel_odom_sub = None
        if self.use_odometry:
            self.wheel_odom_sub = self.create_subscription(
                Odometry, self.wheel_odom_topic, self.odom_callback, 10)
            self.get_logger().info(f'Subscribing to wheel odometry: {self.wheel_odom_topic}')
        else:
            self.get_logger().info('Wheel odometry disabled - using scan-only matching')

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, self.output_odom_topic, 10)

        # State
        self.prev_points = None
        self.current_pose = np.array([0.0, 0.0, 0.0])  # x, y, theta
        self.current_twist = np.array([0.0, 0.0, 0.0])
        self.odom_current = None
        self.odom_previous = None
        self.last_time = 0.0
        self.last_icp_error = 0.01  # Track ICP error for covariance
        
        self.get_logger().info(f'Scan matcher initialized:')
        self.get_logger().info(f'  Scan topic: {self.scan_topic}')
        self.get_logger().info(f'  Output topic: {self.output_odom_topic}')
        self.get_logger().info(f'  Use wheel odom: {self.use_odometry}')

    def odom_callback(self, msg):
        # Extract pose from odometry message
        pose = msg.pose.pose
        theta = self.quaternion_to_yaw(pose.orientation)
        self.odom_current = np.array([pose.position.x, pose.position.y, theta])

    def scan_callback(self, scan_msg):
        points = self.scan_to_points(scan_msg)
        points = self.trim_points(points)

        if self.prev_points is None or len(points) < 10:
            self.prev_points = points
            self.odom_previous = self.odom_current.copy() if self.odom_current is not None else None
            self.last_time = scan_msg.header.stamp.sec + scan_msg.header.stamp.nanosec * 1e-9
            self.get_logger().info('First scan received or insufficient points, initializing...')
            return

        # Initialize delta for case where no odometry is used
        delta = np.array([0.0, 0.0, 0.0])
        
        # Compute initial guess from wheel odometry
        initial_transform = np.eye(3)
        if self.use_odometry and self.odom_current is not None and self.odom_previous is not None:
            delta = self.relative_transform(self.odom_previous, self.odom_current)
            initial_transform = self.transform_matrix(delta)
            self.get_logger().debug(f'Using wheel odom delta: {delta}')

        # Apply initial transform to current points as starting guess for ICP
        if not np.allclose(initial_transform, np.eye(3)):
            # Transform current points by initial guess
            ones = np.ones((len(points), 1))
            homogeneous_points = np.hstack([points, ones])
            transformed_points = (initial_transform @ homogeneous_points.T).T[:, :2]
        else:
            transformed_points = points.copy()

        # Perform ICP alignment
        icp_transform = self.icp_2d(transformed_points, self.prev_points)
        
        # Combine initial guess with ICP result
        total_transform = icp_transform @ initial_transform
        
        # Extract pose change from transform matrix
        dx = total_transform[0, 2]
        dy = total_transform[1, 2]
        dtheta = np.arctan2(total_transform[1, 0], total_transform[0, 0])
        
        # Update current pose
        self.current_pose[0] += dx
        self.current_pose[1] += dy
        self.current_pose[2] += dtheta
        
        # Normalize angle
        self.current_pose[2] = np.arctan2(np.sin(self.current_pose[2]), np.cos(self.current_pose[2]))
        
        self.get_logger().debug(f'ICP delta: dx={dx:.3f}, dy={dy:.3f}, dtheta={dtheta:.3f}')

        # Estimate velocity
        current_time = scan_msg.header.stamp.sec + scan_msg.header.stamp.nanosec * 1e-9
        dt = current_time - self.last_time
        if dt > 0:
            self.current_twist[0] = delta[0] / dt  # linear x
            self.current_twist[1] = delta[1] / dt  # linear y  
            self.current_twist[2] = delta[2] / dt  # angular z
            self.last_time = current_time

        # Publish the updated odometry
        self.publish_odom(scan_msg.header.stamp)

        # Update previous state
        self.prev_points = points
        self.odom_previous = self.odom_current.copy() if self.odom_current is not None else None

    def scan_to_points(self, scan):
        angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))
        ranges = np.array(scan.ranges)

        # Use parameterized range limits
        mask = (np.isfinite(ranges) & 
                (ranges > max(scan.range_min, self.min_range)) & 
                (ranges < min(scan.range_max, self.max_range)))
        
        x = ranges[mask] * np.cos(angles[mask])
        y = ranges[mask] * np.sin(angles[mask])
        return np.stack((x, y), axis=-1)

    def trim_points(self, points):
        """Downsample points by the configured factor"""
        if self.downsample_factor <= 1:
            return points
        return points[np.arange(len(points)) % self.downsample_factor == 0]

    def icp_2d(self, source_points, target_points):
        """
        Align source_points to target_points using ICP.
        Returns 3x3 transform matrix (rotation + translation).
        """
        src = source_points.copy()
        T_total = np.eye(3)

        for iteration in range(self.max_iterations):
            # 1. Find correspondences
            tree = KDTree(target_points)
            distances, indices = tree.query(src)
            
            # Filter correspondences by distance
            valid_mask = distances < self.max_corr_dist
            if np.sum(valid_mask) < 10:  # Need minimum points
                self.get_logger().warn(f'ICP iteration {iteration}: Only {np.sum(valid_mask)} valid correspondences')
                break
                
            src_filtered = src[valid_mask]
            matched_target = target_points[indices[valid_mask]]

            # 2. Compute centroids
            centroid_src = np.mean(src_filtered, axis=0)
            centroid_tgt = np.mean(matched_target, axis=0)

            # 3. Center the clouds
            src_centered = src_filtered - centroid_src
            tgt_centered = matched_target - centroid_tgt

            # 4. Compute covariance and SVD
            H = src_centered.T @ tgt_centered
            U, _, Vt = np.linalg.svd(H)
            R = Vt.T @ U.T

            # Ensure it's a proper rotation (no reflection)
            if np.linalg.det(R) < 0:
                Vt[1, :] *= -1
                R = Vt.T @ U.T

            # 5. Compute translation
            t = centroid_tgt - R @ centroid_src

            # 6. Form transform
            T = np.eye(3)
            T[:2, :2] = R
            T[:2, 2] = t

            # 7. Update cumulative transform and source
            src = (R @ src.T).T + t
            T_total = T @ T_total

            # 8. Check convergence
            mean_error = np.mean(distances[valid_mask])
            self.last_icp_error = mean_error  # Track for covariance
            if mean_error < self.tolerance:
                self.get_logger().debug(f'ICP converged after {iteration+1} iterations, error: {mean_error:.6f}')
                break

        return T_total
    
    def relative_transform(self, prev, curr):
        dx = curr[0] - prev[0]
        dy = curr[1] - prev[1]
        dtheta = curr[2] - prev[2]

        # Transform dx,dy into previous frame
        c, s = cos(-prev[2]), sin(-prev[2])
        dx_local = c * dx - s * dy
        dy_local = s * dx + c * dy

        return np.array([dx_local, dy_local, dtheta])

    def transform_matrix(self, delta):
        dx, dy, theta = delta
        return np.array([
            [cos(theta), -sin(theta), dx],
            [sin(theta),  cos(theta), dy],
            [0,           0,          1]
        ])

    def publish_odom(self, stamp: Time):
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        x, y, theta = self.current_pose
        quat = self.yaw_to_quaternion(theta)

        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.orientation = quat

        # Estimate covariance based on ICP convergence
        pose_uncertainty = max(0.01, self.last_icp_error * 0.1)  # Scale with ICP error
        
        # Add covariance matrix
        odom_msg.pose.covariance = [pose_uncertainty, 0.0, 0.0, 0.0, 0.0, 0.0,  # x
                                   0.0, pose_uncertainty, 0.0, 0.0, 0.0, 0.0,  # y  
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # z (unused)
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # roll (unused)
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # pitch (unused)
                                   0.0, 0.0, 0.0, 0.0, 0.0, pose_uncertainty * 0.5]  # yaw

        # Add twist if available
        odom_msg.twist.twist.linear.x = self.current_twist[0]
        odom_msg.twist.twist.linear.y = self.current_twist[1] 
        odom_msg.twist.twist.angular.z = self.current_twist[2]

        self.odom_pub.publish(odom_msg)

    def yaw_to_quaternion(self, yaw):
        qz = sin(yaw / 2.0)
        qw = cos(yaw / 2.0)
        return Quaternion(x=0.0, y=0.0, z=qz, w=qw)

    def quaternion_to_yaw(self, q):
        return atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * q.z * q.z)

def main(args=None):
    rclpy.init(args=args)
    node = ScanMatcherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
