import numpy as np
from d2oc_algorithm.scoring import uncertainty, travel_cost, information_score


def test_uncertainty_peaks_at_unknown():
    assert uncertainty(0.5) == 1.0
    assert uncertainty(0.0) == 0.0
    assert uncertainty(1.0) == 0.0
    assert abs(uncertainty(0.25) - 0.5) < 1e-9
    assert abs(uncertainty(0.75) - 0.5) < 1e-9


def test_uncertainty_vectorized():
    out = uncertainty(np.array([0.0, 0.5, 1.0]))
    assert np.allclose(out, [0.0, 1.0, 0.0])


def test_travel_cost_monotonic_and_positive():
    assert travel_cost(0.0) > 0.0
    assert travel_cost(10.0) > travel_cost(1.0)


def test_information_score_prefers_near_unknown_unvisited():
    near = information_score(0.5, 0.0, 1.0)
    far = information_score(0.5, 0.0, 20.0)
    assert near > far > 0.0


def test_information_score_penalizes_visited():
    fresh = information_score(0.5, 0.0, 1.0)
    visited = information_score(0.5, 1.0, 1.0)
    assert fresh > visited
    assert visited == 0.0


def test_information_score_zero_for_certain_cells():
    assert information_score(0.0, 0.0, 1.0) == 0.0
    assert information_score(1.0, 0.0, 1.0) == 0.0
