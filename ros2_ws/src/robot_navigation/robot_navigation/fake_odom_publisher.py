#!/usr/bin/env python3
"""
Simple fake odometry publisher for testing SLAM
Publishes zero odometry data for stationary testing
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math


class FakeOdomPublisher(Node):
    def __init__(self):
        super().__init__('fake_odom_publisher')
        
        # Publisher for odometry
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # TF broadcaster for odom->base_link transform
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer to publish at 50Hz
        self.timer = self.create_timer(0.02, self.publish_odom)
        
        self.get_logger().info('Fake odometry publisher started')
    
    def publish_odom(self):
        current_time = self.get_clock().now()
        
        # Create odometry message (all zeros for stationary test)
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        # Position (all zeros)
        odom.pose.pose.position.x = 0.0
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = 0.0
        
        # Orientation (no rotation)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = 0.0
        odom.pose.pose.orientation.w = 1.0
        
        # Velocity (all zeros)
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = 0.0
        
        # Set covariance matrices (small values indicating good confidence)
        # Initialize all to zeros first
        for i in range(36):
            odom.pose.covariance[i] = 0.0
            odom.twist.covariance[i] = 0.0
        
        # Set diagonal elements for pose covariance
        odom.pose.covariance[0] = 0.01   # x variance
        odom.pose.covariance[7] = 0.01   # y variance  
        odom.pose.covariance[14] = 0.01  # z variance
        odom.pose.covariance[21] = 0.01  # roll variance
        odom.pose.covariance[28] = 0.01  # pitch variance
        odom.pose.covariance[35] = 0.01  # yaw variance
        
        # Set diagonal elements for twist covariance
        odom.twist.covariance[0] = 0.01   # vx variance
        odom.twist.covariance[7] = 0.01   # vy variance
        odom.twist.covariance[14] = 0.01  # vz variance  
        odom.twist.covariance[21] = 0.01  # vroll variance
        odom.twist.covariance[28] = 0.01  # vpitch variance
        odom.twist.covariance[35] = 0.01  # vyaw variance
        
        # Publish odometry
        self.odom_pub.publish(odom)
        
        # Broadcast transform from odom to base_link
        transform = TransformStamped()
        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"
        
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    fake_odom_publisher = FakeOdomPublisher()
    
    try:
        rclpy.spin(fake_odom_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        fake_odom_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()