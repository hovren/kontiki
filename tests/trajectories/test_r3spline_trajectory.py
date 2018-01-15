import pytest
import numpy as np

from taser.trajectories import UniformR3SplineTrajectory


@pytest.fixture
def random_r3_spline():
    N = 10
    control_points = np.random.uniform(-5, 5, size=(N, 3))

    traj = UniformR3SplineTrajectory()
    for cp in control_points:
        traj.append_knot(cp)
    return traj, control_points


def test_construct_default():
    traj = UniformR3SplineTrajectory()
    assert traj.dt == 1.0
    assert traj.t0 == 0.0
    assert len(traj) == 0


@pytest.mark.parametrize('args',[
    (1.0, 0.0),
    (0.5, 1.0),
    (0.5, -1.0),
    (1.0, ),
    (0.5, ),
])
def test_construct_params_ok(args):
    traj = UniformR3SplineTrajectory(*args)
    assert len(traj) == 0


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