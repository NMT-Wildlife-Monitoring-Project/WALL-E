"""
entropy_calculator.py
=====================
Component 2 of D2OC Algorithm: Entropy Computation and Candidate Extraction

This module computes Shannon entropy from occupancy probabilities and returns
high-information cells for exploration planning.
"""

import math
from typing import Iterable, List, Sequence, Tuple

import numpy as np


class EntropyCalculator:
	"""
	Compute uncertainty (entropy) from occupancy probabilities.

	Entropy definition for Bernoulli occupancy probability p:
		H(p) = -p*log2(p) - (1-p)*log2(1-p)

	Interpretation:
	  - p = 0.0 or 1.0  -> H = 0.0  (certain)
	  - p = 0.5         -> H = 1.0  (maximum uncertainty)
	"""

	def __init__(self, epsilon: float = 1e-9):
		self.epsilon = float(epsilon)

	def entropy(self, probability: float) -> float:
		"""
		Return Shannon entropy for a single occupancy probability.

		Parameters
		----------
		probability : float in [0.0, 1.0]

		Returns
		-------
		float in [0.0, 1.0]
		"""
		p = float(np.clip(probability, self.epsilon, 1.0 - self.epsilon))
		return float(-p * math.log2(p) - (1.0 - p) * math.log2(1.0 - p))

	def entropy_map(self, occupancy_array: np.ndarray) -> np.ndarray:
		"""
		Vectorized entropy over a full occupancy grid.

		Parameters
		----------
		occupancy_array : np.ndarray
			2D array of occupancy probabilities in [0.0, 1.0]

		Returns
		-------
		np.ndarray
			2D entropy array with same shape as occupancy_array.
		"""
		probs = np.clip(occupancy_array.astype(np.float32), self.epsilon, 1.0 - self.epsilon)
		return -probs * np.log2(probs) - (1.0 - probs) * np.log2(1.0 - probs)

	def find_high_entropy_cells(
		self,
		density_map,
		threshold: float = 0.8,
		min_confidence: float = 0.0,
	) -> List[Tuple[float, float, float]]:
		"""
		Find all cells with entropy >= threshold.

		Parameters
		----------
		density_map : DensityMap-like object
			Must provide fields/methods used below:
			- occupancy (2D np.ndarray)
			- confidence (2D np.ndarray)
			- grid_to_world(col, row) -> (x, y)
		threshold : float
			Minimum entropy to keep a cell as exploration candidate.
		min_confidence : float
			Optional lower bound on map confidence for candidate filtering.

		Returns
		-------
		list[(world_x, world_y, entropy)]
		"""
		threshold = float(np.clip(threshold, 0.0, 1.0))
		min_confidence = float(np.clip(min_confidence, 0.0, 1.0))

		entropy_grid = self.entropy_map(density_map.occupancy)

		if min_confidence > 0.0:
			mask = (entropy_grid >= threshold) & (density_map.confidence >= min_confidence)
		else:
			mask = entropy_grid >= threshold

		rows, cols = np.where(mask)

		high_entropy_cells: List[Tuple[float, float, float]] = []
		for row, col in zip(rows, cols):
			world_x, world_y = density_map.grid_to_world(int(col), int(row))
			high_entropy_cells.append((float(world_x), float(world_y), float(entropy_grid[row, col])))

		return high_entropy_cells

	def rank_cells_by_entropy(
		self,
		cell_list: Sequence[Tuple[float, float, float]],
		density_map=None,
	) -> List[Tuple[float, float, float]]:
		"""
		Sort candidate cells by entropy descending.

		If tuples already include entropy (x, y, entropy), that value is used.
		If tuples are (x, y) and density_map is provided, entropy is computed
		from the map and returned as (x, y, entropy).
		"""
		if not cell_list:
			return []

		first = cell_list[0]
		if len(first) == 3:
			ranked = sorted(cell_list, key=lambda item: item[2], reverse=True)
			return [(float(x), float(y), float(h)) for x, y, h in ranked]

		if len(first) == 2 and density_map is not None:
			enriched: List[Tuple[float, float, float]] = []
			for world_x, world_y in cell_list:
				grid_idx = density_map.world_to_grid(float(world_x), float(world_y))
				if grid_idx is None:
					continue
				col, row = grid_idx
				p = float(density_map.occupancy[row, col])
				enriched.append((float(world_x), float(world_y), self.entropy(p)))
			return sorted(enriched, key=lambda item: item[2], reverse=True)

		raise ValueError(
			"cell_list must be either [(x, y, entropy), ...] or [(x, y), ...] with density_map provided"
		)

	def information_gain_for_cells(
		self,
		cells: Sequence[Tuple[float, float, float]],
	) -> float:
		"""
		Aggregate information gain proxy by summing candidate entropies.

		This helper is optional but useful for debugging and future scoring.
		"""
		if not cells:
			return 0.0
		return float(sum(cell[2] for cell in cells))
