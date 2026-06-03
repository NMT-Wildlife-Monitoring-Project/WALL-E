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
