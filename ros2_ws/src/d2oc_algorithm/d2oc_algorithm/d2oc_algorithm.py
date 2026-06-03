"""Core D2OC decision logic — sim-brain re-architecture (Phase 1).

Pipeline (mirrors d2oc-speedup.py):
  1) Candidate cells = uncertain cells (occupancy > candidate_occ_min),
     excluding confident-free cells and known obstacles.
  2) Score each: information_score(occupancy, visit_freq, distance).
  3) Barycenter = score-weighted centroid of the top-K cells (grid space),
     exponentially smoothed across calls.
  4) NW = nearest KNOWN-FREE cell (0 < occ < free_occ_max) to the barycenter,
     random pick among the nearest `nw_candidates`, optionally costmap-gated.
  5) Return NW as a PoseStamped goal in the map frame.
"""
import numpy as np
from geometry_msgs.msg import PoseStamped

from .scoring import information_score


class D2OCAlgorithm:
    def __init__(
        self,
        candidate_occ_min=0.4,
        free_occ_max=0.5,
        free_occ_min=0.0,
        top_k=10,
        nw_candidates=5,
        gamma=0.1,
        cost_offset=1.5,
        max_candidates=500,
        bary_smoothing=0.3,
        goal_frame_id='map',
        rng=None,
    ):
        self.candidate_occ_min = float(candidate_occ_min)
        self.free_occ_max = float(free_occ_max)
        self.free_occ_min = float(free_occ_min)
        self.top_k = int(top_k)
        self.nw_candidates = int(nw_candidates)
        self.gamma = float(gamma)
        self.cost_offset = float(cost_offset)
        self.max_candidates = int(max_candidates)
        self.bary_smoothing = float(bary_smoothing)
        self.goal_frame_id = goal_frame_id
        self.rng = rng if rng is not None else np.random.default_rng()
        self.bary = None  # running barycenter in grid indices: np.array([col, row])

    def compute_exploration_goal(self, density_map, robot_x, robot_y, visit_map,
                                 costmap=None, stamp=None):
        bary = self._update_barycenter(density_map, robot_x, robot_y, visit_map)
        if bary is None:
            return None
        nw = self._select_next_waypoint(density_map, bary, costmap)
        if nw is None:
            return None
        nw_x, nw_y = density_map.grid_to_world(nw[0], nw[1])
        return self._to_pose_stamped(nw_x, nw_y, stamp=stamp)

    def _update_barycenter(self, density_map, robot_x, robot_y, visit_map):
        occ = density_map.occupancy
        rows, cols = np.where(occ > self.candidate_occ_min)
        if rows.size == 0:
            return None

        if rows.size > self.max_candidates:
            sel = self.rng.choice(rows.size, self.max_candidates, replace=False)
            rows, cols = rows[sel], cols[sel]

        world_x = density_map.origin_x + (cols + 0.5) * density_map.resolution
        world_y = density_map.origin_y + (rows + 0.5) * density_map.resolution
        distance = np.hypot(world_x - robot_x, world_y - robot_y)

        freq = visit_map.frequency(rows, cols)
        score = information_score(occ[rows, cols], freq, distance,
                                  gamma=self.gamma, offset=self.cost_offset)

        if not np.any(score > 0.0):
            score = 1.0 / (distance + 1e-6)

        k = min(self.top_k, score.size)
        top = np.argsort(-score)[:k]
        weights = score[top]
        if weights.sum() <= 0.0:
            weights = np.ones_like(weights)

        bary_col = float(np.average(cols[top], weights=weights))
        bary_row = float(np.average(rows[top], weights=weights))
        raw = np.array([bary_col, bary_row], dtype=float)

        if self.bary is None:
            self.bary = raw
        else:
            a = self.bary_smoothing
            self.bary = (1.0 - a) * self.bary + a * raw
        return self.bary

    def _select_next_waypoint(self, density_map, bary, costmap):
        occ = density_map.occupancy
        free_mask = (occ > self.free_occ_min) & (occ < self.free_occ_max)
        rows, cols = np.where(free_mask)
        if rows.size == 0:
            return None

        d2 = (cols - bary[0]) ** 2 + (rows - bary[1]) ** 2
        order = np.argsort(d2)

        n = min(self.nw_candidates, order.size)
        nearest = order[:n].copy()
        self.rng.shuffle(nearest)
        for idx in nearest:
            c, r = int(cols[idx]), int(rows[idx])
            if self._is_accessible(density_map, costmap, c, r):
                return (c, r)
        c, r = int(cols[order[0]]), int(rows[order[0]])
        return (c, r)

    def _is_accessible(self, density_map, costmap, col, row):
        if costmap is None:
            return True
        world_x, world_y = density_map.grid_to_world(col, row)
        res = costmap.info.resolution
        if res <= 0.0:
            return True
        c = int((world_x - costmap.info.origin.position.x) / res)
        r = int((world_y - costmap.info.origin.position.y) / res)
        if c < 0 or r < 0 or c >= costmap.info.width or r >= costmap.info.height:
            return False
        value = int(costmap.data[r * costmap.info.width + c])
        if value < 0:
            return True
        return value < 65

    def _to_pose_stamped(self, x, y, stamp=None):
        msg = PoseStamped()
        msg.header.frame_id = self.goal_frame_id
        if stamp is not None:
            msg.header.stamp = stamp
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        return msg
