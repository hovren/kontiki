from collections import defaultdict, namedtuple

import numpy as np
import pytest

from taser.trajectories import ConstantTrajectory, LinearTrajectory, SimpleMultiTrajectory, UniformR3SplineTrajectory
from taser.rotations import quat_to_rotation_matrix

ExampleData = namedtuple('ExampleData',
                         ['position', 'velocity', 'acceleration', 'orientation', 'angular_velocity',
                          'min_time', 'max_time'])


def safe_time(trajectory):
    tmin, tmax = trajectory.valid_time

    # Base case: Both finite
    if np.isfinite(tmin) and np.isfinite(tmax):
        t = 0.5 * (tmin + tmax)

    else:
        # At least one is infinite
        # Make sure tmin is not inf or tmax = -inf
        assert tmax > tmin

        if np.isfinite(tmin):  # (a, np.inf) -> t >= a OK
            t = tmin + 1
        elif np.isfinite(tmax):  # (-inf, b) -> t < b is OK
            t = tmax - 1
        else:
            t = 42.  # (-inf, inf) means any time is valid, pick something non-zero

    # Sanity check
    assert np.isfinite(t)
    return t


@pytest.fixture
def trajectory_example(trajectory):
    def make_example(tmin, tmax):
         return ExampleData([],[],[],[],[], tmin, tmax)
    cls = trajectory.__class__

    zero = np.zeros(3)
    q0 = np.array([1, 0, 0, 0])
    if cls == LinearTrajectory:
        example_data = make_example(-np.inf, np.inf)
        # Example positions
        p_ex1 = 0, np.array([-0.2, 0, -0.8])
        p_ex2 = -1, np.array([-0.3, 0, -1.2])
        p_ex3 = 2, zero
        example_data.position.extend([p_ex1, p_ex2, p_ex3])

        # Example velocities
        example_data.velocity.extend([
            (0, np.array([.1, 0, .4])),
            (1, np.array([.1, 0, .4]))
        ])

        # Example accelerations
        example_data.acceleration.extend([
            (0, zero)
        ])

        # Orientations
        example_data.orientation.extend([
            (2, q0),  # t=t0 -> identity
            #(3, np.array([-0.47129323,  0.21391074,  0.        ,  0.85564297])),
            (3, np.array([ 0.97882515,  0.04964659,  0.        ,  0.19858634])),
        ])

        example_data.angular_velocity.extend([
            (0, np.array([.1, 0, .4])),
            (1, np.array([.1, 0, .4]))
        ])

    elif cls == ConstantTrajectory:
        example_data = make_example(-np.inf, np.inf)
        k = np.array([1, 2, 3])
        example_data.position.extend([[0, k], [2, k]])
        example_data.velocity.extend([[0, zero], [1, zero]])
        example_data.acceleration.extend([[0, zero], [1, zero]])
        example_data.orientation.extend([[0, q0], [1, q0]])
        example_data.angular_velocity.extend([[0, zero], [0, zero]])

    elif cls == SimpleMultiTrajectory:
        example_data = make_example(-np.inf, np.inf)
        example_data.position.extend([
            (0.1, np.array([1,5,1])),
            (1.1, np.array([7, 61, 557])),
            (2.2, np.array([17, 71, 567])),
            (3.3, np.array([117, 171, 667])),
            (4.3, np.array([117, 171, 667]))
        ])
        example_data.velocity.extend([])

    elif cls == UniformR3SplineTrajectory:
        from test_r3spline_trajectory import scipy_bspline_for_trajectory, scipy_bspline_valid_time_interval
        control_points = np.vstack([cp for cp in trajectory])
        bspline = scipy_bspline_for_trajectory(trajectory)
        bspline_vel = bspline.derivative(1)
        bspline_acc = bspline.derivative(2)
        t1, t2 = scipy_bspline_valid_time_interval(bspline)

        example_data = make_example(t1, t2)
        times = np.linspace(t1, t2, endpoint=False)
        example_data.position.extend([(t, bspline(t)) for t in times])
        example_data.velocity.extend([(t, bspline_vel(t)) for t in times])
        example_data.acceleration.extend([(t, bspline_acc(t)) for t in times])
        example_data.orientation.extend([(t, q0) for t in times])
        example_data.angular_velocity.extend([(t, np.zeros(3)) for t in times])
    else:
        raise NotImplementedError(f"No example data for {cls} available")

    return trajectory, example_data


def test_translation_return_type(trajectory):
    t = safe_time(trajectory)
    p = trajectory.position(t)
    assert p.shape == (3,)
    v = trajectory.velocity(t)
    assert v.shape == (3,)
    a = trajectory.acceleration(t)
    assert a.shape == (3,)


def test_orientation_return_type(trajectory):
    t = safe_time(trajectory)
    q = trajectory.orientation(t)
    assert q.shape == (4,)
    np.testing.assert_almost_equal(np.linalg.norm(q), 1)
    w = trajectory.angular_velocity(t)
    assert w.shape == (3,)


@pytest.mark.parametrize("modality",
    ["position", "velocity", "acceleration", "orientation", "angular_velocity"])
def test_trajectory_example(trajectory_example, modality):
    trajectory, example_data = trajectory_example
    expected_data = getattr(example_data, modality)
    func = getattr(trajectory, modality)

    assert len(expected_data) > 0, "No test data available"
    for t, x in expected_data:
        xhat = func(t)
        np.testing.assert_almost_equal(x, xhat)


def test_valid_time(trajectory_example):
    trajectory, example_data = trajectory_example

    t1 = trajectory.min_time
    t2 = trajectory.max_time
    assert trajectory.valid_time == (t1, t2)
    assert t1 == example_data.min_time
    assert t2 == example_data.max_time


def test_from_world(trajectory):
    t = safe_time(trajectory)
    Rwt = quat_to_rotation_matrix(trajectory.orientation(t))
    pwt = trajectory.position(t)

    X_world = np.random.uniform(-10, 10, size=3)
    X_traj_expected = Rwt.T @ (X_world - pwt)
    X_traj_actual = trajectory.from_world(X_world, t)
    np.testing.assert_almost_equal(X_traj_actual, X_traj_expected)


def test_to_world(trajectory):
    t = safe_time(trajectory)
    Rwt = quat_to_rotation_matrix(trajectory.orientation(t))
    pwt = trajectory.position(t)

    X_traj = np.random.uniform(-10, 10, size=3)
    X_world_expected = Rwt @ X_traj + pwt
    X_world_actual = trajectory.to_world(X_traj, t)
    np.testing.assert_almost_equal(X_world_actual, X_world_expected)