import pytest
import numpy as np

from taser.measurements import StaticRsCameraMeasurement, PositionMeasurement, GyroscopeMeasurement
from taser.rotations import quat_to_rotation_matrix
from taser.sfm import Landmark, View

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

def test_gyroscope_measurements(trajectory_example):
    trajectory, example_data = trajectory_example

    for t, w in example_data.angular_velocity:
        m = GyroscopeMeasurement(t, w)
        w_hat = m.measure(trajectory)
        np.testing.assert_almost_equal(w_hat, w)