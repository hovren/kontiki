from collections import defaultdict, namedtuple

import numpy as np
import pytest

from taser.trajectories import ConstantTrajectory, LinearTrajectory

ExampleData = namedtuple('ExampleData',
                         ['position', 'velocity', 'acceleration', 'orientation', 'angular_velocity'])

@pytest.fixture
def trajectory_example(simple_trajectory):
    example_data = ExampleData([],[],[],[],[])
    trajectory = simple_trajectory
    cls = trajectory.__class__

    if cls == LinearTrajectory:
        # Example positions
        p_ex1 = 0, np.array([-2, 0, -8])
        p_ex2 = -1, np.array([-3, 0, -12])
        p_ex3 = 2, np.array([0, 0, 0])
        example_data.position.extend([p_ex1, p_ex2, p_ex3])

    return trajectory, example_data


def test_translation_return_type(simple_trajectory):
    p = simple_trajectory.position(0)
    assert p.shape == (3,)
    v = simple_trajectory.velocity(0)
    assert v.shape == (3,)
    a = simple_trajectory.acceleration(0)
    assert a.shape == (3,)


def test_orientation_return_type(simple_trajectory):
    q = simple_trajectory.orientation(0)
    assert q.shape == (4,)
    np.testing.assert_almost_equal(np.linalg.norm(q), 1)
    w = simple_trajectory.angular_velocity(0)
    assert w.shape == (3,)


def test_trajectory_example(trajectory_example):
    trajectory, example_data = trajectory_example

    for t, p in example_data.position:
        phat = trajectory.position(t)
        np.testing.assert_equal(p, phat)
