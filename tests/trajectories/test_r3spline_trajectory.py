import pytest
import numpy as np
from scipy.interpolate import BSpline

from taser.trajectories import UniformR3SplineTrajectory


def scipy_bspline(control_points, dt=1, k=3, t0=0):
    n = len(control_points)
    Nt = k + n + 1
    t = dt * (np.arange(Nt) - k) + t0
    return BSpline(t, control_points, k, extrapolate=False)


def scipy_bspline_valid_time_interval(bspl):
    return bspl.t[bspl.k], bspl.t[-bspl.k - 1]


def scipy_bspline_for_trajectory(trajectory):
    control_points = np.vstack([cp for cp in trajectory])
    return scipy_bspline(control_points, dt=trajectory.dt, t0=trajectory.t0, k=3)

@pytest.fixture
def random_r3_spline():
    N = 10
    control_points = np.random.uniform(-5, 5, size=(N, 3))

    dt = np.random.uniform(0.1, 2.0)
    t0 = np.random.uniform(-2, 2)

    traj = UniformR3SplineTrajectory(dt, t0)
    for cp in control_points:
        traj.append_knot(cp)
    return traj, control_points


def test_construct_default():
    traj = UniformR3SplineTrajectory()
    assert traj.dt == 1.0
    assert traj.t0 == 0.0
    assert len(traj) == 0


@pytest.mark.parametrize('dt,t0',[
    (1.0, 0.0),
    (0.5, 1.0),
    (0.5, -1.0),
    (1.0, None),
    (0.5, None),
])
def test_construct_params_ok(dt, t0):
    if t0 is None:
        traj = UniformR3SplineTrajectory(dt)
        t0 = 0.0  # Expected t0
    else:
        traj = UniformR3SplineTrajectory(dt, t0)
    assert len(traj) == 0
    assert traj.dt == dt
    assert traj.t0 == t0


def test_control_points(random_r3_spline):
    traj, control_points = random_r3_spline

    assert len(traj) == len(control_points)
    actual_control_points = np.vstack([cp for cp in traj])

    np.testing.assert_equal(actual_control_points, control_points)


def test_negative_control_point_indices(random_r3_spline):
    traj, control_points = random_r3_spline
    assert len(control_points) == 10
    assert len(traj) == len(control_points)

    N = len(traj)
    for neg_i in range(-N, 0):
        pos_i = neg_i + N
        neg_cp = traj[neg_i]
        pos_cp = traj[pos_i]
        np.testing.assert_equal(neg_cp, pos_cp)

@pytest.mark.parametrize("i", [-11, -12, 10, 11])
def test_out_of_bounds(random_r3_spline, i):
    traj, _ = random_r3_spline
    with pytest.raises(IndexError):
        cp = traj[i]


def test_set_control_point(random_r3_spline):
    traj, _ = random_r3_spline

    for i in [0, 2, -2]:
        orig_cp = traj[i]
        new_cp = np.random.uniform(-5, 5, size=3)
        traj[i] = new_cp
        np.testing.assert_equal(traj[i], new_cp)


def test_position(random_r3_spline):
    traj, control_points = random_r3_spline

    bspl_sp = scipy_bspline(control_points, dt=traj.dt, t0=traj.t0, k=3)

    times = np.linspace(*scipy_bspline_valid_time_interval(bspl_sp), endpoint=False)

    for t in times:
        pos_sp = bspl_sp(t)
        pos_our = traj.position(t)
        np.testing.assert_almost_equal(pos_our, pos_sp)


def test_velocity(random_r3_spline):
    traj, control_points = random_r3_spline

    bspl_sp = scipy_bspline(control_points, dt=traj.dt, t0=traj.t0, k=3)

    times = np.linspace(*scipy_bspline_valid_time_interval(bspl_sp), endpoint=False)

    vel_func = bspl_sp.derivative(1)
    for t in times:
        vel_sp = vel_func(t)
        vel_our = traj.velocity(t)
        np.testing.assert_almost_equal(vel_our, vel_sp)


def test_acceleration(random_r3_spline):
    traj, control_points = random_r3_spline

    bspl_sp = scipy_bspline(control_points, dt=traj.dt, t0=traj.t0, k=3)

    times = np.linspace(*scipy_bspline_valid_time_interval(bspl_sp), endpoint=False)

    acc_func = bspl_sp.derivative(2)
    for t in times:
        acc_sp = acc_func(t)
        acc_our = traj.acceleration(t)
        np.testing.assert_almost_equal(acc_our, acc_sp)
