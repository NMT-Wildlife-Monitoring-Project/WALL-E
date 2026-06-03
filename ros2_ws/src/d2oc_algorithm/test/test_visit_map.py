import numpy as np
from d2oc_algorithm.visit_map import VisitMap


def test_frequency_zero_before_any_visit():
    vm = VisitMap(rows=20, cols=20, kernel_radius=1)
    freq = vm.frequency(np.array([10]), np.array([10]))
    assert freq.shape == (1,)
    assert freq[0] == 0.0


def test_register_increments_total_and_local_counts():
    vm = VisitMap(rows=20, cols=20, kernel_radius=1)
    vm.register(col=10, row=10)
    assert vm.total == 1.0
    center = vm.frequency(np.array([10]), np.array([10]))[0]
    far = vm.frequency(np.array([2]), np.array([2]))[0]
    assert center > 0.0
    assert far == 0.0


def test_repeated_visits_raise_frequency():
    vm = VisitMap(rows=20, cols=20, kernel_radius=1)
    vm.register(10, 10)
    once = vm.frequency(np.array([10]), np.array([10]))[0]
    vm.register(10, 10)
    twice = vm.frequency(np.array([10]), np.array([10]))[0]
    assert twice > once


def test_register_near_edge_does_not_crash():
    vm = VisitMap(rows=20, cols=20, kernel_radius=2)
    vm.register(col=0, row=0)
    vm.register(col=19, row=19)
    assert vm.total == 2.0


def test_decay_reduces_old_counts():
    vm = VisitMap(rows=20, cols=20, kernel_radius=1, decay=0.5)
    vm.register(10, 10)
    before = vm.counts[10, 10]
    vm.register(2, 2)
    assert vm.counts[10, 10] < before
