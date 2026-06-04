"""Core D2OC decision logic — reachable local-frontier exploration (Phase 1).

Pipeline:
  1) Local window of radius max_goal_distance around the robot.
  2) Flood-fill from the robot through known-free cells -> reachable free.
  3) Frontier = unknown cells adjacent to REACHABLE free.
  4) Score frontiers (info/cost), weighted barycenter of top-K.
  5) Goal = nearest reachable free cell to the barycenter (deterministic).
  6) Hysteresis: hold the current goal until reached or no longer a valid
     reachable frontier, so the goal is stable yet advances as the robot drives.

Observation-only: nothing consumes the goal; Nav2 is not wired.
"""
import math
import numpy as np
from geometry_msgs.msg import PoseStamped

from .scoring import information_score


class D2OCAlgorithm:
    def __init__(
        self,
        candidate_occ_min=0.4,
        free_occ_max=0.5,
        free_occ_min=0.0,
        obstacle_occ_min=0.65,
        top_k=10,
        nw_candidates=5,
        gamma=0.1,
        cost_offset=1.5,
        max_candidates=500,
        bary_smoothing=0.3,
        max_goal_distance=4.0,
        goal_reached_radius=0.5,
        goal_frame_id='map',
        rng=None,
    ):
        self.candidate_occ_min = float(candidate_occ_min)
        self.free_occ_max = float(free_occ_max)
        self.free_occ_min = float(free_occ_min)
        self.obstacle_occ_min = float(obstacle_occ_min)
        self.top_k = int(top_k)
        self.nw_candidates = int(nw_candidates)
        self.gamma = float(gamma)
        self.cost_offset = float(cost_offset)
        self.max_candidates = int(max_candidates)
        self.bary_smoothing = float(bary_smoothing)
        self.max_goal_distance = float(max_goal_distance)
        self.goal_reached_radius = float(goal_reached_radius)
        self.goal_frame_id = goal_frame_id
        self.rng = rng if rng is not None else np.random.default_rng()
        self.current_goal = None  # committed goal as (world_x, world_y) or None

    def compute_exploration_goal(self, density_map, robot_x, robot_y, visit_map,
                                 costmap=None, stamp=None):
        # Hysteresis: keep the committed goal until reached or invalid.
        if self.current_goal is not None:
            gx, gy = self.current_goal
            reached = math.hypot(gx - robot_x, gy - robot_y) <= self.goal_reached_radius
            if not reached and self._goal_still_valid(density_map, gx, gy):
                return self._to_pose_stamped(gx, gy, from_x=robot_x, from_y=robot_y, stamp=stamp)

        goal = self._select_new_goal(density_map, robot_x, robot_y, visit_map)
        self.current_goal = goal
        if goal is None:
            return None
        return self._to_pose_stamped(goal[0], goal[1], from_x=robot_x, from_y=robot_y, stamp=stamp)

    def _select_new_goal(self, density_map, robot_x, robot_y, visit_map):
        occ = density_map.occupancy
        rc = density_map.world_to_grid(robot_x, robot_y)
        if rc is None:
            return None
        rcol, rrow = rc

        win = max(1, int(self.max_goal_distance / density_map.resolution))
        r0 = max(0, rrow - win); r1 = min(density_map.rows, rrow + win + 1)
        c0 = max(0, rcol - win); c1 = min(density_map.cols, rcol + win + 1)
        sub = occ[r0:r1, c0:c1]

        free = (sub > self.free_occ_min) & (sub < self.free_occ_max)
        unknown = (sub >= self.free_occ_max) & (sub < self.obstacle_occ_min)

        reachable = self._flood_fill_free(free, rrow - r0, rcol - c0)
        if not reachable.any():
            return None

        radj = np.zeros_like(reachable)
        radj[1:, :] |= reachable[:-1, :]
        radj[:-1, :] |= reachable[1:, :]
        radj[:, 1:] |= reachable[:, :-1]
        radj[:, :-1] |= reachable[:, 1:]
        frontier = unknown & radj

        fr, fc = np.where(frontier)
        if fr.size == 0:
            return None

        world_x = density_map.origin_x + (fc + c0 + 0.5) * density_map.resolution
        world_y = density_map.origin_y + (fr + r0 + 0.5) * density_map.resolution
        distance = np.hypot(world_x - robot_x, world_y - robot_y)

        freq = visit_map.frequency(fr + r0, fc + c0)
        score = information_score(sub[fr, fc], freq, distance,
                                  gamma=self.gamma, offset=self.cost_offset)
        if not np.any(score > 0.0):
            score = 1.0 / (distance + 1e-6)

        k = min(self.top_k, score.size)
        top = np.argsort(-score)[:k]
        weights = score[top]
        if weights.sum() <= 0.0:
            weights = np.ones_like(weights)
        bcol = float(np.average(fc[top] + c0, weights=weights))
        brow = float(np.average(fr[top] + r0, weights=weights))

        # Goal = nearest REACHABLE free cell to the barycenter (deterministic).
        rr, rcc = np.where(reachable)
        d2 = (rcc + c0 - bcol) ** 2 + (rr + r0 - brow) ** 2
        best = int(np.argmin(d2))
        nw_col = int(rcc[best] + c0)
        nw_row = int(rr[best] + r0)
        return density_map.grid_to_world(nw_col, nw_row)

    def _flood_fill_free(self, free, start_row, start_col):
        reachable = np.zeros_like(free)
        rows, cols = free.shape
        if rows == 0 or cols == 0:
            return reachable
        sr, sc = start_row, start_col
        if not (0 <= sr < rows and 0 <= sc < cols and free[sr, sc]):
            fr, fc = np.where(free)
            if fr.size == 0:
                return reachable
            i = int(np.argmin((fr - start_row) ** 2 + (fc - start_col) ** 2))
            sr, sc = int(fr[i]), int(fc[i])
        stack = [(sr, sc)]
        reachable[sr, sc] = True
        while stack:
            r, c = stack.pop()
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nr, nc = r + dr, c + dc
                if 0 <= nr < rows and 0 <= nc < cols and free[nr, nc] and not reachable[nr, nc]:
                    reachable[nr, nc] = True
                    stack.append((nr, nc))
        return reachable

    def _goal_still_valid(self, density_map, gx, gy):
        cell = density_map.world_to_grid(gx, gy)
        if cell is None:
            return False
        gcol, grow = cell
        occ = density_map.occupancy
        if not (self.free_occ_min < occ[grow, gcol] < self.free_occ_max):
            return False
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nr, nc = grow + dr, gcol + dc
            if 0 <= nr < density_map.rows and 0 <= nc < density_map.cols:
                if self.free_occ_max <= occ[nr, nc] < self.obstacle_occ_min:
                    return True
        return False

    def _to_pose_stamped(self, x, y, from_x=None, from_y=None, stamp=None):
        msg = PoseStamped()
        msg.header.frame_id = self.goal_frame_id
        if stamp is not None:
            msg.header.stamp = stamp
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0
        if from_x is not None and from_y is not None and (x != from_x or y != from_y):
            yaw = math.atan2(y - from_y, x - from_x)
            msg.pose.orientation.z = math.sin(yaw / 2.0)
            msg.pose.orientation.w = math.cos(yaw / 2.0)
        else:
            msg.pose.orientation.w = 1.0
        return msg
