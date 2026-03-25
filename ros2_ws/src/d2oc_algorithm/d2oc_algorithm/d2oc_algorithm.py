"""Core D2OC decision logic (Step 6).

Selects the next exploration goal by combining:
- information value (entropy)
- distance penalty
- accessibility constraints from costmap
"""

import math
from typing import List, Optional, Tuple

from geometry_msgs.msg import PoseStamped


class D2OCAlgorithm:
	"""Density-Driven Optimal Controller exploration policy."""

	def __init__(
		self,
		entropy_calculator,
		entropy_threshold: float = 0.8,
		max_goal_distance: float = 20.0,
		distance_weight: float = 0.1,
		min_confidence: float = 0.3,
		candidate_stride: int = 1,
		max_entropy_candidates: int = 0,
		goal_frame_id: str = 'map',
	):
		self.entropy_calculator = entropy_calculator
		self.entropy_threshold = float(entropy_threshold)
		self.max_goal_distance = float(max_goal_distance)
		self.distance_weight = float(distance_weight)
		self.min_confidence = float(min_confidence)
		self.candidate_stride = max(1, int(candidate_stride))
		self.max_entropy_candidates = max(0, int(max_entropy_candidates))
		self.goal_frame_id = goal_frame_id

	def compute_exploration_goal(
		self,
		density_map,
		robot_x: float,
		robot_y: float,
		costmap=None,
		stamp=None,
	) -> Optional[PoseStamped]:
		"""Compute the best next exploration goal.

		Steps:
		1) Extract high-entropy candidate cells
		2) Filter by distance
		3) Filter by accessibility (costmap + map occupancy)
		4) Score with: score = entropy - distance_weight * distance
		5) Return highest-scoring candidate as PoseStamped
		"""
		candidates = self.entropy_calculator.find_high_entropy_cells(
			density_map,
			threshold=self.entropy_threshold,
			min_confidence=self.min_confidence,
			stride=self.candidate_stride,
			max_cells=self.max_entropy_candidates,
		)
		if not candidates:
			return None

		scored_candidates = []
		for world_x, world_y, entropy_value in candidates:
			distance = math.hypot(world_x - robot_x, world_y - robot_y)
			if distance > self.max_goal_distance:
				continue

			if not self._is_accessible(density_map, costmap, world_x, world_y):
				continue

			score = float(entropy_value) - self.distance_weight * distance
			scored_candidates.append((score, world_x, world_y, entropy_value, distance))

		if not scored_candidates:
			return None

		scored_candidates.sort(key=lambda item: item[0], reverse=True)
		_, goal_x, goal_y, _, _ = scored_candidates[0]
		return self._to_pose_stamped(goal_x, goal_y, stamp=stamp)

	def _is_accessible(self, density_map, costmap, world_x: float, world_y: float) -> bool:
		"""Basic accessibility check using local map evidence and Nav2 costmap."""
		grid_idx = density_map.world_to_grid(world_x, world_y)
		if grid_idx is None:
			return False

		col, row = grid_idx
		occupancy_prob = density_map.get_occupancy(col, row)
		if occupancy_prob >= 0.65:
			return False

		if costmap is None:
			return True

		cost_idx = self._world_to_costmap_indices(costmap, world_x, world_y)
		if cost_idx is None:
			return False

		c_col, c_row = cost_idx
		offset = c_row * costmap.info.width + c_col
		if offset < 0 or offset >= len(costmap.data):
			return False

		value = int(costmap.data[offset])
		if value < 0:
			return False
		if value >= 65:
			return False
		return True

	def _world_to_costmap_indices(self, costmap, world_x: float, world_y: float) -> Optional[Tuple[int, int]]:
		"""Convert world coordinates to OccupancyGrid cell indices."""
		origin_x = costmap.info.origin.position.x
		origin_y = costmap.info.origin.position.y
		resolution = costmap.info.resolution
		if resolution <= 0.0:
			return None

		col = int((world_x - origin_x) / resolution)
		row = int((world_y - origin_y) / resolution)

		if col < 0 or row < 0:
			return None
		if col >= costmap.info.width or row >= costmap.info.height:
			return None
		return col, row

	def _to_pose_stamped(self, x: float, y: float, stamp=None) -> PoseStamped:
		"""Create a PoseStamped goal in map frame."""
		msg = PoseStamped()
		msg.header.frame_id = self.goal_frame_id
		if stamp is not None:
			msg.header.stamp = stamp

		msg.pose.position.x = float(x)
		msg.pose.position.y = float(y)
		msg.pose.position.z = 0.0
		msg.pose.orientation.w = 1.0
		return msg
