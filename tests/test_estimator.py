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
        estimator.add_measurement(m)


def test_add_camera_measurement(estimator, camera_measurements):
    _test_add_measurement(estimator, camera_measurements)


def test_add_simple_measurements(estimator, simple_measurements):
    _test_add_measurement(estimator, simple_measurements)


def test_add_imu_measurements(estimator, imu_measurements):
    _test_add_measurement(estimator, imu_measurements)


def test_solve_simple_nocrash(estimator, simple_measurements):
    _test_add_measurement(estimator, simple_measurements)
    summary = estimator.solve()
    print(summary.FullReport())
    assert summary.num_parameters > 0


def test_solve_camera_nocrash(estimator, camera_measurements):
    _test_add_measurement(estimator, camera_measurements)
    summary = estimator.solve()
    print(summary.FullReport())
    assert summary.num_parameters > 0


def test_trajectory_lock(trajectory, simple_measurements):
    estimator_unlocked = TrajectoryEstimator(trajectory)
    for m in simple_measurements:
        estimator_unlocked.add_measurement(m)

    summary_unlocked = estimator_unlocked.solve()
    assert summary_unlocked.num_parameters > 0, "Measurements generated no parameters"
    print('U', summary_unlocked.num_parameters_reduced)

    estimator_locked = TrajectoryEstimator(trajectory)
    trajectory.locked = True
    for m in simple_measurements:
        estimator_locked.add_measurement(m)

    summary_locked = estimator_locked.solve()
    print('L', summary_locked.num_parameters_reduced)

    print(summary_unlocked.FullReport())
    print(summary_locked.FullReport())

    assert summary_locked.num_parameters_reduced == 0, "Not locked"