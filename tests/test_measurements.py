import pytest
import numpy as np

from taser.measurements import StaticRsCameraMeasurement
from taser.rotations import quat_to_rotation_matrix
from taser.sfm import Landmark, View

def test_static_old(camera, trajectory):
    t0 = 0.5
    y_true = np.array([np.random.uniform(0, camera.cols), np.random.uniform(0, camera.rows)])
    t = t0 + y_true[1] * camera.readout / camera.rows
    z = np.random.uniform(2, 100)
    X_camera = z * camera.unproject(y_true)
    X_trajectory = X_camera # The same for now
    X_world = trajectory.to_world(X_trajectory, t)

    v = View(0, t0)
    lm = Landmark()
    lm.inverse_depth = 1 / z
    obs = v.create_observation(lm, *y_true)
    lm.reference = obs


    m = StaticRsCameraMeasurement(camera, lm, obs)
    yhat = m.project(trajectory)
    np.testing.assert_almost_equal(yhat, y_true)


def test_static(small_sfm):
    views, trajectory, camera = small_sfm

    # Take the first landmark
    landmarks = {obs.landmark for v in views for obs in v.observations}
    lm = next(_ for _ in landmarks)

    # Make sure the measurements agree
    assert len(lm.observations) >= 2
    for obs in lm.observations[1:]:
        m = StaticRsCameraMeasurement(camera, lm, obs)
        yhat = m.project(trajectory)
        np.testing.assert_almost_equal(yhat, obs.uv)