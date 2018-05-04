from collections import defaultdict, namedtuple

import numpy as np
import pytest

from kontiki.utils import safe_time, safe_time_span
from kontiki.trajectories import UniformR3SplineTrajectory, UniformSO3SplineTrajectory, UniformSE3SplineTrajectory, SplitTrajectory
from kontiki.rotations import quat_to_rotation_matrix

from kontiki import TrajectoryEstimator
from kontiki.measurements import PositionMeasurement

ExampleData = namedtuple('ExampleData',
                         ['position', 'velocity', 'acceleration', 'orientation', 'angular_velocity',
                          'min_time', 'max_time'])

@pytest.fixture
def trajectory_example(trajectory):
    def make_example(tmin, tmax):
         return ExampleData([],[],[],[],[], tmin, tmax)
    cls = trajectory.__class__

    zero = np.zeros(3)
    q0 = np.array([1, 0, 0, 0])
    
    if cls == UniformR3SplineTrajectory:
        from test_spline_trajectories import scipy_bspline_for_trajectory, scipy_bspline_valid_time_interval
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
    elif cls == UniformSO3SplineTrajectory:
        from test_spline_trajectories import scipy_bspline_for_trajectory, scipy_bspline_valid_time_interval
        # Extract time information
        bspline_DONTUSE = scipy_bspline_for_trajectory(trajectory)
        t1, t2 = scipy_bspline_valid_time_interval(bspline_DONTUSE)
        example_data = make_example(t1, t2)
        times = np.linspace(t1, t2, endpoint=False)

        # Note: We don't have any reference implementation for quaternion cubic b-splines
        # to test against, and constructing examples by hand is difficult.
        # Accept that the orientation test will fail for now

        # Constant angular velocity
        w, axis = np.deg2rad(10), np.array([1., 0, 1])
        axis /= np.linalg.norm(axis)
        def axis_angle_to_quat(n, theta):
            q = np.empty(4)
            q[0] = np.cos(theta / 2)
            q[1:] = np.sin(theta / 2) * n
            return q

        #example_data.orientation.extend([(t, axis_angle_to_quat(axis, w*t)) for t in times])
        example_data.angular_velocity.extend([(t, w * axis) for t in times])

        # Position modalities should be zero for this trajectory type
        example_data.position.extend([(t, zero) for t in times])
        example_data.velocity.extend([(t, zero) for t in times])
        example_data.acceleration.extend([(t, zero) for t in times])
    elif cls == SplitTrajectory:
        _, r3_example = trajectory_example(trajectory.R3_spline)
        _, so3_example = trajectory_example(trajectory.SO3_spline)
        tmin = max(r3_example.min_time, so3_example.min_time)
        tmax = min(r3_example.max_time, so3_example.max_time)
        example_data = make_example(tmin, tmax)
        example_data.position.extend(r3_example.position)
        example_data.velocity.extend(r3_example.velocity)
        example_data.acceleration.extend(r3_example.acceleration)
        example_data.orientation.extend(so3_example.orientation)
        example_data.angular_velocity.extend(so3_example.angular_velocity)
    elif cls == UniformSE3SplineTrajectory:
        pytest.xfail("We have no good way to produce data for this test case")

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

    if modality == 'orientation' and type(trajectory) in (UniformSO3SplineTrajectory, SplitTrajectory):
        pytest.xfail('Orientation examples for SO3 and split missing')

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


def test_velocity_numerical(trajectory):
    import scipy.misc
    t = safe_time(trajectory)
    dt = 1e-3
    vel_num = scipy.misc.derivative(trajectory.position, t, dx=dt, n=1)
    vel_actual = trajectory.velocity(t)
    np.testing.assert_almost_equal(vel_actual, vel_num, decimal=3)


def test_acceleration_numerical(trajectory):
    import scipy.misc

    if type(trajectory) == UniformSE3SplineTrajectory:
        pytest.xfail("SE3 fails because second order derivative is not the same as body acceleration")

    t = safe_time(trajectory)
    dt = 1e-3
    acc_num = scipy.misc.derivative(trajectory.velocity, t, dx=dt, n=1)
    acc_actual = trajectory.acceleration(t)
    np.testing.assert_almost_equal(acc_actual, acc_num, decimal=3)


def test_angular_velocity_numerical(trajectory):
    import scipy.misc
    from kontiki.rotations import quat_mult, quat_conj
    t = safe_time(trajectory)
    dt = 1e-3
    q = trajectory.orientation(t)
    dq_num = scipy.misc.derivative(trajectory.orientation, t, dx=dt, n=1)
    w_num = (2 * quat_mult(dq_num, quat_conj(q)))[1:]
    print()
    print(f'Python: q={q}')
    print(f'Python: dq={dq_num}')
    w_actual = trajectory.angular_velocity(t)
    np.testing.assert_almost_equal(w_actual, w_num, decimal=4)


def test_locking(trajectory):
    assert not trajectory.locked
    trajectory.locked = True
    assert trajectory.locked


def test_clone(trajectory):
    if type(trajectory) == UniformSO3SplineTrajectory:
        pytest.xfail('Test mutates position, which this trajectory does not have')

    t1, t2 = safe_time_span(trajectory, 4)
    times = np.arange(t1, t2, 0.01)

    def sample(trajectory):
        return np.vstack([trajectory.position(t) for t in times])

    orig_pos = sample(trajectory)

    # Clone
    cloned_trajectory = trajectory.clone()
    cloned_pos = sample(cloned_trajectory)
    np.testing.assert_equal(orig_pos, cloned_pos)

    # Change original
    meas = [PositionMeasurement(t, np.random.uniform(-4, 1, size=3)) for t in np.arange(t1, t2, 0.1)]
    estimator = TrajectoryEstimator(trajectory)
    for m in meas:
        estimator.add_measurement(m)
    estimator.solve(max_iterations=10)
    new_pos = sample(trajectory)
    assert np.linalg.norm(new_pos - orig_pos) > 1

    # Check that clone did NOT change
    new_cloned = sample(cloned_trajectory)
    np.testing.assert_equal(new_cloned, orig_pos)

