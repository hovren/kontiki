import pytest

import numpy as np

from taser.trajectory_estimator import TrajectoryEstimator

@pytest.fixture
def estimator(trajectory):
    return TrajectoryEstimator(trajectory)

def test_same_trajectory(trajectory):
    "Test that we can create estimators for all trajectory types"
    estimator = TrajectoryEstimator(trajectory)
    assert estimator.trajectory is trajectory


def test_solve_empty(estimator):
    summary = estimator.solve()
    print(summary.FullReport())
    assert summary.num_parameters == 0


def _test_add_measurement(estimator, measurements):
    for m in measurements:
        try:
            estimator.add_measurement(m)
        except ValueError:
            pass  # This means only that the measurement time doesn't match the trajectory


def test_add_camera_measurement(estimator, camera_measurements):
    _test_add_measurement(estimator, camera_measurements)


def test_add_simple_measurements(estimator, simple_measurements):
    _test_add_measurement(estimator, simple_measurements)


def test_solve_simple_nocrash(estimator, simple_measurements):
    _test_add_measurement(estimator, simple_measurements)
    summary = estimator.solve()
    print(summary.FullReport())
    assert summary.num_parameters > 0
