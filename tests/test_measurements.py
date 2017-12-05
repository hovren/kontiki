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
        m = StaticRsCameraMeasurement(camera, lm, obs)
        yhat = m.project(trajectory)
        np.testing.assert_almost_equal(yhat, obs.uv)