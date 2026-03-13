"""ROS2 integration node for D2OC exploration (Step 7)."""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from .d2oc_algorithm import D2OCAlgorithm
from .density_map import DensityMap
from .entropy_calculator import EntropyCalculator


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
	"""Convert quaternion to yaw (Z-axis rotation)."""
	siny_cosp = 2.0 * (w * z + x * y)
	cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
	return math.atan2(siny_cosp, cosy_cosp)


class D2OCNode(Node):
	"""Main node orchestrating D2OC mapping + goal selection."""

	def __init__(self):
		super().__init__('d2oc_explorer')

		self._declare_parameters()
		self._load_parameters()

		self.density_map = DensityMap(
			width=self.grid_width,
			height=self.grid_height,
			resolution=self.grid_resolution,
		)
		self.entropy_calculator = EntropyCalculator()
		self.algorithm = D2OCAlgorithm(
			entropy_calculator=self.entropy_calculator,
			entropy_threshold=self.entropy_threshold,
			max_goal_distance=self.max_goal_distance,
			distance_weight=self.distance_weight,
			min_confidence=self.min_confidence,
			goal_frame_id='map',
		)

		self.latest_costmap = None
		self.robot_x = None
		self.robot_y = None
		self.robot_theta = None
		self.tf_buffer = Buffer()
		self.tf_listener = TransformListener(self.tf_buffer, self)

		self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 20)
		self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 20)
		self.create_subscription(OccupancyGrid, self.costmap_topic, self.costmap_callback, 10)

		self.goal_publisher = self.create_publisher(PoseStamped, self.exploration_goal_topic, 10)
		self.map_publisher = self.create_publisher(OccupancyGrid, self.density_map_topic, 10)

		timer_period = 1.0 / max(self.publish_frequency, 1e-6)
		self.create_timer(timer_period, self.timer_callback)

		self.get_logger().info('D2OC node initialized')

	def _declare_parameters(self):
		self.declare_parameter('grid.width', 100.0)
		self.declare_parameter('grid.height', 100.0)
		self.declare_parameter('grid.resolution', 0.2)

		self.declare_parameter('algorithm.entropy_threshold', 0.8)
		self.declare_parameter('algorithm.max_goal_distance', 20.0)
		self.declare_parameter('algorithm.distance_weight', 0.1)
		self.declare_parameter('algorithm.min_confidence', 0.3)

		self.declare_parameter('sensor.scan_confidence', 0.7)

		self.declare_parameter('publish.frequency', 1.0)
		self.declare_parameter('publish.enable_density_map', True)

		self.declare_parameter('topics.scan', '/scan')
		self.declare_parameter('topics.odometry', '/odometry/filtered')
		self.declare_parameter('topics.costmap', '/local_costmap/costmap')
		self.declare_parameter('topics.exploration_goal', '/exploration/goal')
		self.declare_parameter('topics.density_map', '/d2oc/density_map')

	def _load_parameters(self):
		self.grid_width = float(self.get_parameter('grid.width').value)
		self.grid_height = float(self.get_parameter('grid.height').value)
		self.grid_resolution = float(self.get_parameter('grid.resolution').value)

		self.entropy_threshold = float(self.get_parameter('algorithm.entropy_threshold').value)
		self.max_goal_distance = float(self.get_parameter('algorithm.max_goal_distance').value)
		self.distance_weight = float(self.get_parameter('algorithm.distance_weight').value)
		self.min_confidence = float(self.get_parameter('algorithm.min_confidence').value)

		self.scan_confidence = float(self.get_parameter('sensor.scan_confidence').value)

		self.publish_frequency = float(self.get_parameter('publish.frequency').value)
		self.enable_density_map = bool(self.get_parameter('publish.enable_density_map').value)

		self.scan_topic = str(self.get_parameter('topics.scan').value)
		self.odom_topic = str(self.get_parameter('topics.odometry').value)
		self.costmap_topic = str(self.get_parameter('topics.costmap').value)
		self.exploration_goal_topic = str(self.get_parameter('topics.exploration_goal').value)
		self.density_map_topic = str(self.get_parameter('topics.density_map').value)

	def scan_callback(self, msg: LaserScan):
		target_frame = 'map'
		source_frame = msg.header.frame_id if msg.header.frame_id else 'base_link'

		try:
			transform = self.tf_buffer.lookup_transform(
				target_frame,
				source_frame,
				msg.header.stamp,
				timeout=Duration(seconds=0.1),
			)
		except TransformException as exc:
			self.get_logger().debug(f'TF lookup failed for scan integration: {exc}')
			return

		t = transform.transform.translation
		q = transform.transform.rotation
		scan_x = float(t.x)
		scan_y = float(t.y)
		scan_theta = quaternion_to_yaw(q.x, q.y, q.z, q.w)

		self.density_map.update_from_scan(
			msg,
			robot_x=scan_x,
			robot_y=scan_y,
			robot_theta=scan_theta,
			confidence=self.scan_confidence,
		)

	def odom_callback(self, msg: Odometry):
		self.robot_x = float(msg.pose.pose.position.x)
		self.robot_y = float(msg.pose.pose.position.y)

		q = msg.pose.pose.orientation
		self.robot_theta = quaternion_to_yaw(q.x, q.y, q.z, q.w)

	def costmap_callback(self, msg: OccupancyGrid):
		self.latest_costmap = msg

	def timer_callback(self):
		if self.robot_x is None or self.robot_y is None:
			self.get_logger().debug('Waiting for odometry before computing goals')
			return

		goal = self.algorithm.compute_exploration_goal(
			density_map=self.density_map,
			robot_x=self.robot_x,
			robot_y=self.robot_y,
			costmap=self.latest_costmap,
			stamp=self.get_clock().now().to_msg(),
		)

		if goal is not None:
			self.goal_publisher.publish(goal)
			self.get_logger().info(
				f'Published exploration goal: x={goal.pose.position.x:.2f}, y={goal.pose.position.y:.2f}'
			)
		else:
			self.get_logger().warn('No valid exploration goal found this cycle')

		if self.enable_density_map:
			grid_msg = self.density_map.to_occupancy_grid(
				stamp=self.get_clock().now().to_msg(),
				frame_id='map',
			)
			self.map_publisher.publish(grid_msg)


def main(args=None):
	rclpy.init(args=args)
	node = D2OCNode()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()
