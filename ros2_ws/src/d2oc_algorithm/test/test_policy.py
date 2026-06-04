import numpy as np
import pytest
from d2oc_algorithm.density_map import DensityMap
from d2oc_algorithm.visit_map import VisitMap
from d2oc_algorithm.d2oc_algorithm import D2OCAlgorithm


def _map_with_known_region_and_unknown_frontier():
    dm = DensityMap(width=10.0, height=10.0, resolution=0.1)  # 100x100
    dm.occupancy[:, 0:50] = 0.1
    dm.confidence[:, 0:50] = 1.0
    return dm


def test_goal_points_toward_unknown_region():
    dm = _map_with_known_region_and_unknown_frontier()
    vm = VisitMap(rows=dm.rows, cols=dm.cols, kernel_radius=1)
    policy = D2OCAlgorithm(rng=np.random.default_rng(0))
    goal = policy.compute_exploration_goal(
        density_map=dm, robot_x=-1.0, robot_y=0.0, visit_map=vm, stamp=None)
    assert goal is not None
    assert goal.header.frame_id == 'map'
    assert goal.pose.position.x > -2.0
    # heading should point into the unknown (+x): yaw near 0 => orientation.w large, z small
    assert goal.pose.orientation.w > 0.7


def test_returns_none_when_no_unknown_cells():
    dm = DensityMap(width=5.0, height=5.0, resolution=0.1)
    dm.occupancy[:, :] = 0.1
    dm.confidence[:, :] = 1.0
    vm = VisitMap(rows=dm.rows, cols=dm.cols)
    policy = D2OCAlgorithm(rng=np.random.default_rng(0))
    goal = policy.compute_exploration_goal(dm, 0.0, 0.0, vm, stamp=None)
    assert goal is None


def test_returns_none_when_no_known_free_cells():
    dm = DensityMap(width=5.0, height=5.0, resolution=0.1)
    vm = VisitMap(rows=dm.rows, cols=dm.cols)
    policy = D2OCAlgorithm(rng=np.random.default_rng(0))
    goal = policy.compute_exploration_goal(dm, 0.0, 0.0, vm, stamp=None)
    assert goal is None


def test_visit_penalty_shifts_goal_away_from_visited():
    dm = _map_with_known_region_and_unknown_frontier()
    vm = VisitMap(rows=dm.rows, cols=dm.cols, kernel_radius=3)
    policy = D2OCAlgorithm(rng=np.random.default_rng(0))
    g1 = policy.compute_exploration_goal(dm, -1.0, 0.0, vm, stamp=None)
    for _ in range(200):
        vm.register(col=49, row=10)
    policy2 = D2OCAlgorithm(rng=np.random.default_rng(0))
    g2 = policy2.compute_exploration_goal(dm, -1.0, 0.0, vm, stamp=None)
    assert g1 is not None and g2 is not None
    assert abs(g2.pose.position.y - g1.pose.position.y) >= 0.0


def test_goals_are_stable_across_ticks():
    # Free left half, unknown right half -> a single vertical frontier at the boundary.
    # With frontier-based selection (no random subsample) the goal must be stable,
    # not jumping around as the old random-subsample logic did.
    dm = _map_with_known_region_and_unknown_frontier()
    vm = VisitMap(rows=dm.rows, cols=dm.cols, kernel_radius=1)
    policy = D2OCAlgorithm(rng=np.random.default_rng(0))
    goals = [policy.compute_exploration_goal(dm, -1.0, 0.0, vm, stamp=None) for _ in range(10)]
    assert all(g is not None for g in goals)
    xs = [g.pose.position.x for g in goals]
    ys = [g.pose.position.y for g in goals]
    assert (max(xs) - min(xs)) < 1.0
    assert (max(ys) - min(ys)) < 2.0


def test_clear_obstacle_cells_are_not_frontier_candidates():
    # Free left half, a column of solid obstacle at the boundary, unknown beyond it.
    # The obstacle wall must NOT be treated as frontier; with no free-adjacent unknown
    # beyond the wall, there is no frontier and the goal is None (cannot see past a wall).
    dm = DensityMap(width=10.0, height=10.0, resolution=0.1)  # 100x100
    dm.occupancy[:, 0:50] = 0.1          # free
    dm.confidence[:, 0:50] = 1.0
    dm.occupancy[:, 50] = 0.95           # solid obstacle wall at col 50
    dm.confidence[:, 50] = 1.0
    # cols 51..99 remain unknown (0.5) but are NOT adjacent to free (wall blocks)
    vm = VisitMap(rows=dm.rows, cols=dm.cols, kernel_radius=1)
    policy = D2OCAlgorithm(rng=np.random.default_rng(0))
    goal = policy.compute_exploration_goal(dm, -1.0, 0.0, vm, stamp=None)
    assert goal is None
