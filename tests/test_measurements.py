import pytest
import numpy as np

from taser.measurements import StaticRsCameraMeasurement
from taser.rotations import quat_to_rotation_matrix
from taser.sfm import Landmark, View

def test_static(small_sfm):
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
