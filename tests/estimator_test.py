import pytest

import numpy as np

from taser.trajectory_estimator import TrajectoryEstimator
from taser.trajectories import ConstantTrajectory, LinearTrajectory

@pytest.mark.parametrize("traj_cls",
                         [ConstantTrajectory, LinearTrajectory])
def test_create(traj_cls):
    "Test that we can create estimators for all trajectory types"
    trajectory = traj_cls(np.array([1, 2, 3]))
    estimator = TrajectoryEstimator(trajectory)
    assert estimator.trajectory is trajectory