import pytest
import numpy as np

from taser.measurements import StaticRsCameraMeasurement, LiftingRsCameraMeasurement, NewtonRsCameraMeasurement, \
    PositionMeasurement, GyroscopeMeasurement, AccelerometerMeasurement
from taser.rotations import quat_to_rotation_matrix
from taser.sfm import Landmark, View
from taser.utils import safe_time_span, safe_time
from taser.rotations import quat_to_rotation_matrix

from trajectories.test_general import trajectory_example

projection_types = [StaticRsCameraMeasurement, LiftingRsCameraMeasurement, NewtonRsCameraMeasurement]
imu_measurement_types = [AccelerometerMeasurement, GyroscopeMeasurement]

@pytest.mark.parametrize('cls', projection_types)
def test_rscamera_measurements(cls, small_sfm):
    # NOTE: If this test fails, first try to clear the pytest cache using
    #  python3 -m pytest --cache-clear

    views, trajectory, camera = small_sfm

    # Take the first landmark
    landmarks = {obs.landmark for v in views for obs in v.observations}

    # Make sure the measurements agree
    for lm in landmarks:
        assert len(lm.observations) >= 2
        for obs in lm.observations[1:]:
            m = cls(camera, obs)
            yhat = m.project(trajectory)
            np.testing.assert_almost_equal(yhat, obs.uv)

# Newton method should handle noise in the projection
# Beware that this doesn't seem to catch faulty derivatives for the camera
def test_newton_rscamera_measurements_with_noise(small_sfm):
    # NOTE: If this test fails, first try to clear the pytest cache using
    #  python3 -m pytest --cache-clear

    views, trajectory, camera = small_sfm

    # Take the first landmark
    landmarks = {obs.landmark for v in views for obs in v.observations}

    # The projection error should be below half a row, because that is the threshold we use to terminate the Newton algorithm
    for lm in landmarks:
        assert len(lm.observations) >= 2
        for obs in lm.observations[1:]:
            uv_org = obs.uv
            obs.uv = obs.uv + np.random.normal(0, 2.0, size=2)
            m = NewtonRsCameraMeasurement(camera, obs)
            yhat = m.project(trajectory)
            row_diff = yhat[1] - uv_org[1]
            assert np.abs(row_diff) <= 0.5


@pytest.mark.parametrize('cls', projection_types)
def test_rscamera_measurements_attribute_access(cls, camera):
    lm = Landmark()
    views = [View(i, i/30) for i in range(2)]

    def random_point():
        return np.array([np.random.uniform(0, camera.cols), np.random.uniform(0, camera.rows)])

    ref, obs = [v.create_observation(lm, random_point()) for v in views]
    lm.reference = ref

    m = StaticRsCameraMeasurement(camera, obs)
    assert m.camera is camera
    assert m.observation is obs


def test_camera_errors_size(trajectory, camera_measurements):
    for m in camera_measurements:
        e = m.error(trajectory)
        if issubclass(type(m), LiftingRsCameraMeasurement):
            assert e.size == 3
        else:
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


@pytest.mark.parametrize('mcls', imu_measurement_types)
def test_imu_measurement_same_imu(mcls, imu):
    t = 1.0
    m = mcls(imu, t, np.random.uniform(-1, 1, size=3))
    print(imu, m.imu)
    assert m.imu is imu


@pytest.mark.parametrize('mcls', imu_measurement_types)
def test_imu_measurement_time_offset(mcls, imu, split_trajectory):
    t = safe_time(split_trajectory)
    d = np.random.uniform(-imu.max_time_offset, imu.max_time_offset)
    v = np.random.uniform(-1, 1, size=3)
    m1 = mcls(imu, t, v)
    y1 = m1.measure(split_trajectory)

    imu.time_offset = d
    m2 = mcls(imu, t - d, v)
    y2 = m2.measure(split_trajectory)
    np.testing.assert_equal(y1, y2)


@pytest.mark.parametrize('mcls', projection_types)
def test_camera_measurement_time_offset(mcls, camera, split_trajectory):
    t1, t2 = safe_time_span(split_trajectory, 1)
    t1 += camera.max_time_offset
    t2 -= camera.max_time_offset

    d = np.random.uniform(-camera.max_time_offset, camera.max_time_offset)

    lm = Landmark()
    lm.inverse_depth = np.random.uniform(0.01, 1)
    views = [View(i, t) for i, t in enumerate([t1, t1+0.23])]
    ref, obs = [v.create_observation(lm, np.random.uniform(100, 900, size=2)) for v in views]
    lm.reference = ref

    m1 = mcls(camera, obs)
    y1 = m1.measure(split_trajectory)

    new_lm = Landmark()
    new_lm.inverse_depth = lm.inverse_depth
    new_views = [View(v.frame_nr, v.t0 - d) for v in views]
    new_ref, new_obs = [v.create_observation(new_lm, o.uv) for v, o in zip(new_views, [ref, obs])]
    new_lm.reference = new_ref

    camera.time_offset = d
    m2 = mcls(camera, new_obs)
    y2 = m2.measure(split_trajectory)
    np.testing.assert_almost_equal(y1, y2)
