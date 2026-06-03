"""ROS2 integration node for D2OC exploration (Step 7)."""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from .d2oc_algorithm import D2OCAlgorithm
from .density_map import DensityMap


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
	"""Convert quaternion to yaw (Z-axis rotation)."""
	siny_cosp = 2.0 * (w * z + x * y)
	cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
	return math.atan2(siny_cosp, cosy_cosp)


def stamp_to_nanoseconds(stamp) -> int:
	"""Convert ROS builtin time stamp to nanoseconds."""
	return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


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
		from .visit_map import VisitMap
		self.visit_map = VisitMap(
			rows=self.density_map.rows,
			cols=self.density_map.cols,
			kernel_radius=self.visit_kernel_radius,
			decay=self.visit_decay,
		)
		self.algorithm = D2OCAlgorithm(
			candidate_occ_min=self.candidate_occ_min,
			free_occ_max=self.free_occ_max,
			free_occ_min=self.free_occ_min,
			top_k=self.top_k,
			nw_candidates=self.nw_candidates,
			gamma=self.gamma,
			cost_offset=self.cost_offset,
			max_candidates=self.max_candidates,
			bary_smoothing=self.bary_smoothing,
			goal_frame_id=self.map_frame,
		)

		self.latest_costmap = None
		self.robot_x = None
		self.robot_y = None
		self.robot_theta = None
		self.tf_buffer = Buffer()
		self.tf_listener = TransformListener(self.tf_buffer, self)
		self._last_scan_tf_warning_time = None
		self._last_scan_drop_warning_time = None
		self._last_density_map_publish_time = None

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
		self.declare_parameter('grid.resolution', 0.1)

		# --- policy (sim-brain) ---
		self.declare_parameter('policy.candidate_occ_min', 0.4)
		self.declare_parameter('policy.free_occ_max', 0.5)
		self.declare_parameter('policy.free_occ_min', 0.0)
		self.declare_parameter('policy.top_k', 10)
		self.declare_parameter('policy.nw_candidates', 5)
		self.declare_parameter('policy.gamma', 0.1)
		self.declare_parameter('policy.cost_offset', 1.5)
		self.declare_parameter('policy.max_candidates', 500)
		self.declare_parameter('policy.bary_smoothing', 0.3)
		# --- visit map ---
		self.declare_parameter('visit.kernel_radius', 1)
		self.declare_parameter('visit.decay', 0.0)
		# --- robot pose frame ---
		self.declare_parameter('frames.map', 'map')
		self.declare_parameter('frames.base_link', 'base_link')

		self.declare_parameter('sensor.scan_confidence', 0.6)
		self.declare_parameter('sensor.enable_odom_scan_fallback', False)
		self.declare_parameter('sensor.max_tf_fallback_age_sec', 0.05)

		self.declare_parameter('publish.frequency', 1.0)
		self.declare_parameter('publish.enable_density_map', True)
		self.declare_parameter('publish.density_map_frequency', 0.5)

		self.declare_parameter('topics.scan', '/scan')
		self.declare_parameter('topics.odometry', '/odometry/filtered')
		self.declare_parameter('topics.costmap', '/local_costmap/costmap')
		self.declare_parameter('topics.exploration_goal', '/exploration/goal')
		self.declare_parameter('topics.density_map', '/d2oc/density_map')

	def _load_parameters(self):
		self.grid_width = float(self.get_parameter('grid.width').value)
		self.grid_height = float(self.get_parameter('grid.height').value)
		self.grid_resolution = float(self.get_parameter('grid.resolution').value)

		self.candidate_occ_min = float(self.get_parameter('policy.candidate_occ_min').value)
		self.free_occ_max = float(self.get_parameter('policy.free_occ_max').value)
		self.free_occ_min = float(self.get_parameter('policy.free_occ_min').value)
		self.top_k = int(self.get_parameter('policy.top_k').value)
		self.nw_candidates = int(self.get_parameter('policy.nw_candidates').value)
		self.gamma = float(self.get_parameter('policy.gamma').value)
		self.cost_offset = float(self.get_parameter('policy.cost_offset').value)
		self.max_candidates = int(self.get_parameter('policy.max_candidates').value)
		self.bary_smoothing = float(self.get_parameter('policy.bary_smoothing').value)
		self.visit_kernel_radius = int(self.get_parameter('visit.kernel_radius').value)
		self.visit_decay = float(self.get_parameter('visit.decay').value)
		self.map_frame = str(self.get_parameter('frames.map').value)
		self.base_link_frame = str(self.get_parameter('frames.base_link').value)

		self.scan_confidence = float(self.get_parameter('sensor.scan_confidence').value)
		self.enable_odom_scan_fallback = bool(
			self.get_parameter('sensor.enable_odom_scan_fallback').value
		)
		self.max_tf_fallback_age_sec = float(
			self.get_parameter('sensor.max_tf_fallback_age_sec').value
		)

		self.publish_frequency = float(self.get_parameter('publish.frequency').value)
		self.enable_density_map = bool(self.get_parameter('publish.enable_density_map').value)
		self.density_map_frequency = float(self.get_parameter('publish.density_map_frequency').value)

		self.scan_topic = str(self.get_parameter('topics.scan').value)
		self.odom_topic = str(self.get_parameter('topics.odometry').value)
		self.costmap_topic = str(self.get_parameter('topics.costmap').value)
		self.exploration_goal_topic = str(self.get_parameter('topics.exploration_goal').value)
		self.density_map_topic = str(self.get_parameter('topics.density_map').value)

	def _lookup_robot_pose(self):
		"""Return (x, y, yaw) of base_link in the map frame via TF, or None."""
		try:
			tf = self.tf_buffer.lookup_transform(
				self.map_frame, self.base_link_frame, Time(),
				timeout=Duration(seconds=0.1))
		except TransformException:
			return None
		t = tf.transform.translation
		q = tf.transform.rotation
		return float(t.x), float(t.y), quaternion_to_yaw(q.x, q.y, q.z, q.w)

	def scan_callback(self, msg: LaserScan):
		target_frame = 'map'
		source_frame = msg.header.frame_id if msg.header.frame_id else 'base_link'

		scan_x = None
		scan_y = None
		scan_theta = None

		try:
			transform = self.tf_buffer.lookup_transform(
				target_frame,
				source_frame,
				msg.header.stamp,
				timeout=Duration(seconds=0.1),
			)
			t = transform.transform.translation
			q = transform.transform.rotation
			scan_x = float(t.x)
			scan_y = float(t.y)
			scan_theta = quaternion_to_yaw(q.x, q.y, q.z, q.w)
		except TransformException as exc:
			try:
				latest_transform = self.tf_buffer.lookup_transform(
					target_frame,
					source_frame,
					Time(),
					timeout=Duration(seconds=0.1),
				)
				scan_stamp_ns = stamp_to_nanoseconds(msg.header.stamp)
				tf_stamp_ns = stamp_to_nanoseconds(latest_transform.header.stamp)
				age_ns = abs(scan_stamp_ns - tf_stamp_ns)
				max_age_ns = int(max(self.max_tf_fallback_age_sec, 0.0) * 1e9)

				if age_ns <= max_age_ns:
					t = latest_transform.transform.translation
					q = latest_transform.transform.rotation
					scan_x = float(t.x)
					scan_y = float(t.y)
					scan_theta = quaternion_to_yaw(q.x, q.y, q.z, q.w)
				else:
					now = self.get_clock().now()
					if (
						self._last_scan_drop_warning_time is None
						or (now - self._last_scan_drop_warning_time).nanoseconds > int(5e9)
					):
						self.get_logger().warn(
							f'Dropping scan: latest TF {source_frame}->{target_frame} differs from scan time by '
							f'{age_ns / 1e9:.3f}s (limit {self.max_tf_fallback_age_sec:.3f}s)'
						)
						self._last_scan_drop_warning_time = now
					return
			except TransformException:
				if (
					self.enable_odom_scan_fallback
					and self.robot_x is not None
					and self.robot_y is not None
					and self.robot_theta is not None
				):
					scan_x = self.robot_x
					scan_y = self.robot_y
					scan_theta = self.robot_theta
					now = self.get_clock().now()
					if (
						self._last_scan_tf_warning_time is None
						or (now - self._last_scan_tf_warning_time).nanoseconds > int(5e9)
					):
						self.get_logger().warn(
							f'TF lookup failed for scan integration ({exc}); using odometry pose fallback'
						)
						self._last_scan_tf_warning_time = now
				else:
					now = self.get_clock().now()
					if (
						self._last_scan_drop_warning_time is None
						or (now - self._last_scan_drop_warning_time).nanoseconds > int(5e9)
					):
						self.get_logger().warn(
							f'Dropping scan: no valid TF {source_frame}->{target_frame} (exact or latest). '
							'Enable sensor.enable_odom_scan_fallback only if odometry is in map frame.'
						)
						self._last_scan_drop_warning_time = now
					return

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
		pose = self._lookup_robot_pose()
		if pose is None:
			self.get_logger().debug(
				f'Waiting for {self.map_frame}->{self.base_link_frame} TF before computing goals')
			return
		self.robot_x, self.robot_y, self.robot_theta = pose

		cell = self.density_map.world_to_grid(self.robot_x, self.robot_y)
		if cell is not None:
			self.visit_map.register(col=cell[0], row=cell[1])

		goal = self.algorithm.compute_exploration_goal(
			density_map=self.density_map,
			robot_x=self.robot_x,
			robot_y=self.robot_y,
			visit_map=self.visit_map,
			costmap=self.latest_costmap,
			stamp=self.get_clock().now().to_msg(),
		)

		if goal is not None:
			self.goal_publisher.publish(goal)
			self.get_logger().info(
				f'Published exploration goal: x={goal.pose.position.x:.2f}, '
				f'y={goal.pose.position.y:.2f}')
		else:
			self.get_logger().warn('No valid exploration goal found this cycle')

		self._maybe_publish_density_map()

	def _maybe_publish_density_map(self):
		if not (self.enable_density_map and self.density_map_frequency > 0.0):
			return
		now = self.get_clock().now()
		if self._last_density_map_publish_time is not None:
			elapsed_ns = (now - self._last_density_map_publish_time).nanoseconds
			if elapsed_ns < int(1e9 / self.density_map_frequency):
				return
		grid_msg = self.density_map.to_occupancy_grid(stamp=now.to_msg(), frame_id=self.map_frame)
		self.map_publisher.publish(grid_msg)
		self._last_density_map_publish_time = now


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
