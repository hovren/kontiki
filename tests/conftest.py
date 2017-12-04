import pytest

import numpy as np

from fixtures.camera_fixtures import *

from taser.trajectories import LinearTrajectory, ConstantTrajectory

trajectory_classes = [
    LinearTrajectory,
#    ConstantTrajectory,
]

@pytest.fixture(params=trajectory_classes)
def simple_trajectory(request):
    "Handcrafted 'simple' trajectory"
    cls = request.param

    if cls == LinearTrajectory:
        t0 = 2
        k = np.array([1, 0, 4])
        trajectory = cls(t0, k)
    elif cls == ConstantTrajectory:
        k = np.array([1, 2, 3])
        trajectory = cls(k)
    else:
        raise ValueError(f"Fixture simple_trajectory not available for {cls}")
    return trajectory
