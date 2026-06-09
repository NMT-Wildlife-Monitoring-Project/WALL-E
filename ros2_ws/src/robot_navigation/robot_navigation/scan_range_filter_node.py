"""scan_range_filter_node — republish /scan with near (in-footprint) returns removed.

Why this exists: a cable/mount ~3.5 cm behind the lidar produces laser returns
that sit permanently INSIDE the robot footprint and flicker across the
nav2 collision_monitor `min_points` threshold, intermittently zeroing
/cmd_vel_safe -> jittery motion in BOTH teleop and Nav2.

This node subscribes to the raw scan, replaces any finite return closer than
`min_range` with +inf (nothing real can be that close — it is inside the robot),
and republishes on `output_topic`. Only collision_monitor consumes the filtered
topic; slam_toolbox / rf2o / costmap keep using the raw /scan, so localization
is unaffected.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from .scan_filter import filter_close_returns


class ScanRangeFilter(Node):
    def __init__(self):
        super().__init__('scan_range_filter')
        self.declare_parameter('min_range', 0.18)
        self.declare_parameter('input_topic', 'scan')
        self.declare_parameter('output_topic', 'scan_collision')
        self.min_range = float(self.get_parameter('min_range').value)
        in_topic = str(self.get_parameter('input_topic').value)
        out_topic = str(self.get_parameter('output_topic').value)

        # Match the lidar's sensor QoS (best-effort) so both the subscription
        # and collision_monitor's subscription are QoS-compatible.
        self.pub = self.create_publisher(LaserScan, out_topic, qos_profile_sensor_data)
        self.sub = self.create_subscription(
            LaserScan, in_topic, self.cb, qos_profile_sensor_data)
        self.get_logger().info(
            f'scan_range_filter: dropping returns < {self.min_range:.3f} m '
            f'({in_topic} -> {out_topic})')

    def cb(self, msg: LaserScan):
        msg.ranges = filter_close_returns(list(msg.ranges), self.min_range)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanRangeFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
