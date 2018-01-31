import pytest
import numpy as np

from taser.utils import safe_time
from taser.sensors import BasicImu, ConstantBiasImu

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

