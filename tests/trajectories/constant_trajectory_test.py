import pytest

import numpy as np

import taser
from taser.trajectories._constant_trajectory import ConstantTrajectory

def test_vector_add():
    k = np.array([1., 2.5, 4.])
    traj = ConstantTrajectory(k)
    np.testing.assert_equal(traj.constant, k)