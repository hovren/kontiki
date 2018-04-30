import pytest
import numpy as np

from kontiki.utils import safe_time
from kontiki.sensors import BasicImu, ConstantBiasImu
from kontiki import TrajectoryEstimator
from kontiki.measurements import GyroscopeMeasurement, AccelerometerMeasurement

# --- General tests -----------------------------

def test_has_acceleration(imu, trajectory):
    t = safe_time(trajectory)
    acc = imu.accelerometer(trajectory, t)


def test_has_gyroscope(imu, trajectory):
    t = safe_time(trajectory)
    w = imu.gyroscope(trajectory, t)

# --- Constant bias tests -----------------------------

def random_bias():
    return np.random.uniform(-1, 1, size=3)


def test_constructor_empty():
    imu = ConstantBiasImu()
    np.testing.assert_equal(imu.accelerometer_bias, 0)
    np.testing.assert_equal(imu.gyroscope_bias, 0)


def test_constructor_full():
    abias = random_bias()
    gbias = random_bias()
    imu = ConstantBiasImu(abias, gbias)

    np.testing.assert_equal(imu.accelerometer_bias, abias)
    np.testing.assert_equal(imu.gyroscope_bias, gbias)


def test_change_bias():
    imu = ConstantBiasImu()

    abias = random_bias()
    gbias = random_bias()
    imu.accelerometer_bias = abias
    imu.gyroscope_bias = gbias

    np.testing.assert_equal(imu.accelerometer_bias, abias)
    np.testing.assert_equal(imu.gyroscope_bias, gbias)


def test_set_bias_locks():
    imu = ConstantBiasImu()
    assert imu.gyroscope_bias_locked
    assert imu.accelerometer_bias_locked

    imu.gyroscope_bias_locked = False
    assert not imu.gyroscope_bias_locked
    imu.gyroscope_bias_locked = True
    assert imu.gyroscope_bias_locked

    imu.accelerometer_bias_locked = False
    assert not imu.accelerometer_bias_locked
    imu.accelerometer_bias_locked = True
    assert imu.accelerometer_bias_locked


def test_locks_effective(trajectory):
    imu = ConstantBiasImu()
    imu.accelerometer_bias_locked = True
    imu.gyroscope_bias_locked = True
    t = safe_time(trajectory)
    ma = AccelerometerMeasurement(imu, t, np.array([5, 6, 2]))
    mg = GyroscopeMeasurement(imu, t, np.array([1, 2, 3]))

    estimator_locked = TrajectoryEstimator(trajectory)
    estimator_locked.add_measurement(ma)
    estimator_locked.add_measurement(mg)
    summary_locked = estimator_locked.solve(max_iterations=2)

    imu.accelerometer_bias_locked = False
    imu.gyroscope_bias_locked = False
    estimator_unlocked = TrajectoryEstimator(trajectory)
    estimator_unlocked.add_measurement(ma)
    estimator_unlocked.add_measurement(mg)
    summary_unlocked = estimator_unlocked.solve()

    assert summary_unlocked.num_parameters_reduced == summary_locked.num_parameters_reduced + (2 * 3)

