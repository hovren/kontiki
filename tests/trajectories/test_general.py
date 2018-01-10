from collections import defaultdict, namedtuple

import numpy as np
import pytest

from taser.trajectories import ConstantTrajectory, LinearTrajectory, SimpleMultiTrajectory
from taser.rotations import quat_to_rotation_matrix

ExampleData = namedtuple('ExampleData',
                         ['position', 'velocity', 'acceleration', 'orientation', 'angular_velocity'])

@pytest.fixture
def trajectory_example(trajectory):
    example_data = ExampleData([],[],[],[],[])
    cls = trajectory.__class__

    zero = np.zeros(3)
    q0 = np.array([1, 0, 0, 0])
    if cls == LinearTrajectory:
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

    if cls == ConstantTrajectory:
        k = np.array([1, 2, 3])
        example_data.position.extend([[0, k], [2, k]])
        example_data.velocity.extend([[0, zero], [1, zero]])
        example_data.acceleration.extend([[0, zero], [1, zero]])
        example_data.orientation.extend([[0, q0], [1, q0]])
        example_data.angular_velocity.extend([[0, zero], [0, zero]])

    return trajectory, example_data


def test_translation_return_type(trajectory):
    p = trajectory.position(0)
    assert p.shape == (3,)
    v = trajectory.velocity(0)
    assert v.shape == (3,)
    a = trajectory.acceleration(0)
    assert a.shape == (3,)


def test_orientation_return_type(trajectory):
    q = trajectory.orientation(0)
    assert q.shape == (4,)
    np.testing.assert_almost_equal(np.linalg.norm(q), 1)
    w = trajectory.angular_velocity(0)
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


def test_from_world(trajectory):
    t = 1.0
    Rwt = quat_to_rotation_matrix(trajectory.orientation(t))
    pwt = trajectory.position(t)

    X_world = np.random.uniform(-10, 10, size=3)
    X_traj_expected = Rwt.T @ (X_world - pwt)
    X_traj_actual = trajectory.from_world(X_world, t)
    np.testing.assert_almost_equal(X_traj_actual, X_traj_expected)


def test_to_world(trajectory):
    t = 1.0
    Rwt = quat_to_rotation_matrix(trajectory.orientation(t))
    pwt = trajectory.position(t)

    X_traj = np.random.uniform(-10, 10, size=3)
    X_world_expected = Rwt @ X_traj + pwt
    X_world_actual = trajectory.to_world(X_traj, t)
    np.testing.assert_almost_equal(X_world_actual, X_world_expected)