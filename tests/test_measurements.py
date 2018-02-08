import pytest
import numpy as np

from taser.measurements import StaticRsCameraMeasurement, PositionMeasurement, GyroscopeMeasurement, AccelerometerMeasurement
from taser.rotations import quat_to_rotation_matrix
from taser.sfm import Landmark, View
from taser.utils import safe_time_span
from taser.rotations import quat_to_rotation_matrix

from trajectories.test_general import trajectory_example

def test_static(small_sfm):
    # NOTE: If this test fails, first try to clear the pytest cache using
    #  python3 -m pytest --cache-clear

    views, trajectory, camera = small_sfm

    # Take the first landmark
    landmarks = {obs.landmark for v in views for obs in v.observations}
    lm = next(_ for _ in landmarks)

    # Make sure the measurements agree
    assert len(lm.observations) >= 2
    for obs in lm.observations[1:]:
        m = StaticRsCameraMeasurement(camera, obs)
        yhat = m.project(trajectory)
        np.testing.assert_almost_equal(yhat, obs.uv)

        
def test_static_attribute_access(camera):
    lm = Landmark()
    views = [View(i, i/30) for i in range(2)]

    ref, obs = [v.create_observation(lm, np.random.uniform(0, camera.cols), np.random.uniform(0, camera.rows))
                for v in views]
    lm.reference = ref

    m = StaticRsCameraMeasurement(camera, obs)
    assert m.camera is camera
    assert m.observation is obs


def test_camera_errors_size(trajectory, camera_measurements):
    for m in camera_measurements:
        e = m.error(trajectory)
        assert e.size == 2

def test_position_measurements(trajectory_example):
    trajectory, example_data = trajectory_example

    expected_positions = example_data.position
    for t, x in expected_positions:
        m = PositionMeasurement(t, x)
        xhat = m.measure(trajectory)
        np.testing.assert_almost_equal(xhat, x)

def test_gyroscope_measurements(trajectory, imu):
    times = np.linspace(*safe_time_span(trajectory, 3.0), num=10, endpoint=False)

    def true_gyro(t):
        q = trajectory.orientation(t)
        R = quat_to_rotation_matrix(q)
        w_world = trajectory.angular_velocity(t)
        w_body = R.T @ w_world
        return w_body

    for t in times:
        w = true_gyro(t)
        m = GyroscopeMeasurement(imu, t, w)
        w_hat = m.measure(trajectory)
        try:
            w_hat -= imu.gyroscope_bias
        except AttributeError:
            pass # No bias to remove
        np.testing.assert_almost_equal(w_hat, w)


def test_accelerometer_measurements(trajectory, imu):
    times = np.linspace(*safe_time_span(trajectory, 3.0), num=10, endpoint=False)

    def true_acc(t):
        q = trajectory.orientation(t)
        acc_world = trajectory.acceleration(t)
        R = quat_to_rotation_matrix(q)
        gravity = np.array([0, 0, 9.80665])
        acc = R.T @ (acc_world - gravity)
        return acc

    # Currently fails for ConstantBiasImu since we don't take bias into account
    for t in times:
        acc = true_acc(t)
        m = AccelerometerMeasurement(imu, t, acc)
        acc_hat = m.measure(trajectory)
        try:
            acc_hat -= imu.accelerometer_bias
        except AttributeError:
            pass # No bias to remove
        np.testing.assert_almost_equal(acc_hat, acc)


@pytest.mark.parametrize('mcls', [AccelerometerMeasurement, GyroscopeMeasurement])
def test_imu_measurement_same_imu(mcls, imu):
    t = 1.0
    m = mcls(imu, t, np.random.uniform(-1, 1, size=3))
    print(imu, m.imu)
    assert m.imu is imu