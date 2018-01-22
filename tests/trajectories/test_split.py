import pytest
import numpy as np

from taser.trajectories import SplitTrajectory, UniformR3SplineTrajectory, UniformSO3SplineTrajectory
from taser.utils import safe_time

def test_default_constructor():
    trajectory = SplitTrajectory()
    assert trajectory.R3_spline.dt == 1.0
    assert trajectory.R3_spline.t0 == 0.0
    assert trajectory.SO3_spline.dt == 1.0
    assert trajectory.SO3_spline.t0 == 0.0

    with pytest.raises(ValueError):
        t1, t2 = trajectory.valid_time


def test_dt_constructor():
    r3_dt = np.random.uniform(0.05, 2.0)
    so3_dt = np.random.uniform(0.05, 2.0)

    trajectory = SplitTrajectory(r3_dt, so3_dt)
    assert trajectory.R3_spline.dt == r3_dt
    assert trajectory.SO3_spline.dt == so3_dt


def test_full_constructor():
    r3_dt = np.random.uniform(0.05, 2.0)
    so3_dt = np.random.uniform(0.05, 2.0)
    r3_t0 = np.random.uniform(-4, 4)
    so3_t0 = np.random.uniform(-4, 4)

    trajectory = SplitTrajectory(r3_dt, so3_dt, r3_t0, so3_t0)
    assert trajectory.R3_spline.dt == r3_dt
    assert trajectory.SO3_spline.dt == so3_dt
    assert trajectory.R3_spline.t0 == r3_t0
    assert trajectory.SO3_spline.t0 == so3_t0


def test_from_splines_constructor():
    r3_dt = np.random.uniform(0.05, 2.0)
    so3_dt = np.random.uniform(0.05, 2.0)
    r3_t0 = np.random.uniform(-4, 4)
    so3_t0 = np.random.uniform(-4, 4)
    r3_traj = UniformR3SplineTrajectory(r3_dt, r3_t0)
    so3_traj = UniformSO3SplineTrajectory(so3_dt, so3_t0)

    trajectory = SplitTrajectory(r3_traj, so3_traj)
    assert trajectory.R3_spline is r3_traj
    assert trajectory.SO3_spline is so3_traj
    assert trajectory.R3_spline.dt == r3_dt
    assert trajectory.SO3_spline.dt == so3_dt
    assert trajectory.R3_spline.t0 == r3_t0
    assert trajectory.SO3_spline.t0 == so3_t0
