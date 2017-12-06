import pytest

import numpy as np

from taser.trajectory_estimator import TrajectoryEstimator

def test_create(trajectory):
    "Test that we can create estimators for all trajectory types"
    estimator = TrajectoryEstimator(trajectory)
    assert estimator.trajectory is trajectory


def _test_add_measurement(trajectory, measurements):
    estimator = TrajectoryEstimator(trajectory)
    for m in measurements:
        estimator.add_measurement(m)


def test_add_camera_measurement(trajectory, camera_measurements):
    _test_add_measurement(trajectory, camera_measurements)


def test_add_simple_measurements(trajectory, simple_measurements):
    _test_add_measurement(trajectory, simple_measurements)