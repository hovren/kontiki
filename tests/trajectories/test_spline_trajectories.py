import pytest
import numpy as np
from scipy.interpolate import BSpline

from taser.trajectories import UniformR3SplineTrajectory, UniformSO3SplineTrajectory

spline_classes = (UniformR3SplineTrajectory, UniformSO3SplineTrajectory)

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
    new_control_point_func = lambda: np.random.uniform(-5, 5, size=3)

    dt = np.random.uniform(0.1, 2.0)
    t0 = np.random.uniform(-2, 2)

    traj = UniformR3SplineTrajectory(dt, t0)
    for cp in control_points:
        traj.append_knot(cp)
    return traj, control_points, new_control_point_func


@pytest.fixture
def random_so3_spline():
    N = 10
    control_points = np.random.uniform(-3, 3, size=(N, 4))
    control_points /= np.linalg.norm(control_points, axis=1).reshape(-1, 1)

    def new_control_point_func():
        q = np.random.uniform(-3, 3, size=4)
        return q / np.linalg.norm(q)

    dt = np.random.uniform(0.1, 2.0)
    t0 = np.random.uniform(-2, 2)

    traj = UniformSO3SplineTrajectory(dt, t0)
    for i, cp in enumerate(control_points):
        # Make sure the sequence does not flip sign
        if i > 0:
            prev = traj[i-1]
            if np.dot(prev, cp) < 0:
                cp *= -1
        traj.append_knot(cp)
    return traj, control_points, new_control_point_func


@pytest.fixture(params=spline_classes)
def random_spline(request):
    cls = request.param
    try:
        spline = {
            UniformR3SplineTrajectory: random_r3_spline,
            UniformSO3SplineTrajectory: random_so3_spline,
        }[cls]()
        return spline
    except KeyError:
        raise ValueError(f"No fixture available for {cls}")


# ---- General spline tests -------------------------------- #

@pytest.mark.parametrize("cls", spline_classes)
def test_construct_default(cls):
    traj = cls()
    assert traj.dt == 1.0
    assert traj.t0 == 0.0
    assert len(traj) == 0


@pytest.mark.parametrize("cls", spline_classes)
@pytest.mark.parametrize('dt,t0',[
    (1.0, 0.0),
    (0.5, 1.0),
    (0.5, -1.0),
    (1.0, None),
    (0.5, None),
])
def test_construct_params_ok(cls, dt, t0):
    if t0 is None:
        traj = cls(dt)
        t0 = 0.0  # Expected t0
    else:
        traj = cls(dt, t0)
    assert len(traj) == 0
    assert traj.dt == dt
    assert traj.t0 == t0


def test_control_points(random_spline):
    traj, control_points, *_ = random_spline

    assert len(traj) == len(control_points)
    actual_control_points = np.vstack([cp for cp in traj])

    np.testing.assert_equal(actual_control_points, control_points)


def test_negative_control_point_indices(random_spline):
    traj, control_points, *_ = random_spline
    assert len(control_points) == 10
    assert len(traj) == len(control_points)

    N = len(traj)
    for neg_i in range(-N, 0):
        pos_i = neg_i + N
        neg_cp = traj[neg_i]
        pos_cp = traj[pos_i]
        np.testing.assert_equal(neg_cp, pos_cp)


def test_out_of_bounds(random_spline):
    traj, *_ = random_spline
    N = len(traj)
    for i in (-(N+1), -(N+2), N, N + 1):
        with pytest.raises(IndexError):
            cp = traj[i]


def test_set_control_point(random_spline):
    traj, control_points, new_cp_func, *_ = random_spline

    for i in [0, 2, -2]:
        orig_cp = traj[i]
        new_cp = new_cp_func()
        traj[i] = new_cp
        np.testing.assert_equal(traj[i], new_cp)


# ---- Specific UniformR3Spline tests -------------------------------- #

def test_r3_position(random_r3_spline):
    traj, control_points, *_ = random_r3_spline

    bspl_sp = scipy_bspline(control_points, dt=traj.dt, t0=traj.t0, k=3)

    times = np.linspace(*scipy_bspline_valid_time_interval(bspl_sp), endpoint=False)

    for t in times:
        pos_sp = bspl_sp(t)
        pos_our = traj.position(t)
        np.testing.assert_almost_equal(pos_our, pos_sp)


def test_r3_velocity(random_r3_spline):
    traj, control_points, *_ = random_r3_spline

    bspl_sp = scipy_bspline(control_points, dt=traj.dt, t0=traj.t0, k=3)

    times = np.linspace(*scipy_bspline_valid_time_interval(bspl_sp), endpoint=False)

    vel_func = bspl_sp.derivative(1)
    for t in times:
        vel_sp = vel_func(t)
        vel_our = traj.velocity(t)
        np.testing.assert_almost_equal(vel_our, vel_sp)


def test_r3_acceleration(random_r3_spline):
    traj, control_points, *_ = random_r3_spline

    bspl_sp = scipy_bspline(control_points, dt=traj.dt, t0=traj.t0, k=3)

    times = np.linspace(*scipy_bspline_valid_time_interval(bspl_sp), endpoint=False)

    acc_func = bspl_sp.derivative(2)
    for t in times:
        acc_sp = acc_func(t)
        acc_our = traj.acceleration(t)
        np.testing.assert_almost_equal(acc_our, acc_sp)


# ---- Specific UniformSO3Spline tests -------------------------------- #

def test_so3_require_unit_quaternion():
    traj = UniformSO3SplineTrajectory()
    traj.append_knot(np.array([1., 0., 0., 0.]))  # OK (norm=1)
    with pytest.raises(ValueError):
        traj.append_knot(np.array(1., 1., 1., 1.))  # Fail
