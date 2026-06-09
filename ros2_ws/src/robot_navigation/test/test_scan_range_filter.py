"""Tests for the scan range filter (drops returns inside the robot footprint).

The collision_monitor was stuttering because a cable ~3.5 cm behind the lidar
produces returns that sit permanently inside the footprint and flicker across
min_points. filter_close_returns() replaces any finite return closer than
min_range with +inf (a no-return), so collision_monitor never sees self-hits.
"""
import math

from robot_navigation.scan_filter import filter_close_returns


def test_drops_finite_returns_below_threshold():
    out = filter_close_returns([0.05, 0.10, 0.17, 0.18, 0.50, 1.00], 0.18)
    assert out[0] == math.inf      # 5 cm cable -> dropped
    assert out[1] == math.inf
    assert out[2] == math.inf
    assert out[3] == 0.18          # exactly at threshold -> kept
    assert out[4] == 0.50          # real obstacle -> kept
    assert out[5] == 1.00


def test_preserves_inf_and_nan():
    out = filter_close_returns([math.inf, float('nan'), 0.30], 0.18)
    assert out[0] == math.inf
    assert math.isnan(out[1])
    assert out[2] == 0.30


def test_zero_threshold_is_passthrough():
    vals = [0.01, 0.5, 2.0]
    assert filter_close_returns(vals, 0.0) == vals


def test_returns_new_list_does_not_mutate_input():
    vals = [0.05, 0.5]
    out = filter_close_returns(vals, 0.18)
    assert vals == [0.05, 0.5]     # input untouched
    assert out == [math.inf, 0.5]
