import pytest

import numpy as np

from taser.trajectory_estimator import TrajectoryEstimator

def test_create(simple_trajectory):
    "Test that we can create estimators for all trajectory types"
    estimator = TrajectoryEstimator(simple_trajectory)
    assert estimator.trajectory is simple_trajectory