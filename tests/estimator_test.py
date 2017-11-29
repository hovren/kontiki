import pytest

import numpy as np

from taser._trajectory_estimator import TrajectoryEstimator
from taser.trajectories._constant_trajectory import ConstantTrajectory

def test_create():
    trajectory = ConstantTrajectory(np.array([1, 2, 3]))
    estimator = TrajectoryEstimator(trajectory)
    assert estimator.trajectory is trajectory

    summary = estimator.solve()
    print(summary)
    print(summary.FullReport())
