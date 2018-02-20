import pytest

import numpy as np
from numpy.testing import assert_equal, assert_almost_equal
from conftest import relative_pose

from taser.rotations import quat_to_rotation_matrix

def test_relative_pose_init(sensor):
    q_ct, p_ct = sensor.relative_pose
    assert_equal(p_ct, np.zeros(3))
    assert_equal(q_ct, [1, 0, 0, 0])


def test_relative_pose_set_get(sensor):
    q_ct, p_ct = relative_pose()
    sensor.relative_pose = q_ct, p_ct

    qhat_ct, phat_ct = sensor.relative_pose
    assert_equal(qhat_ct, q_ct)
    assert_equal(phat_ct, p_ct)


def test_relative_pose_argument_order(sensor):
    q_ct, p_ct = relative_pose()
    sensor.relative_pose = q_ct, p_ct # OK

    with pytest.raises(TypeError):
        sensor.relative_pose = p_ct, q_ct # Error


def test_from_trajectory(sensor):
    q_ct, p_ct = sensor.relative_pose
    R_ct = quat_to_rotation_matrix(q_ct)

    X_trajectory = np.random.uniform(-3, 3, size=3)
    X_camera_true = R_ct @ X_trajectory + p_ct
    X_camera_actual = sensor.from_trajectory(X_trajectory)
    assert_almost_equal(X_camera_actual, X_camera_true)


def test_to_trajectory(sensor):
    q_ct, p_ct = sensor.relative_pose
    print(q_ct, p_ct)
    R_ct = quat_to_rotation_matrix(q_ct)

    X_camera = np.random.uniform(-3, 3, size=3)
    X_trajectory_true = R_ct.T @ (X_camera - p_ct)
    X_trajectory_actual = sensor.to_trajectory(X_camera)
    assert_almost_equal(X_trajectory_actual, X_trajectory_true)


def test_time_offset_set_get(sensor):
    assert sensor.time_offset == 0
    new_offset = 0.05
    sensor.time_offset = new_offset
    assert sensor.time_offset == new_offset


def test_time_offset_out_of_bounds(sensor):
    sensor.max_time_offset = 0.1
    sensor.time_offset = 0.05  # OK
    with pytest.raises(ValueError):
        sensor.time_offset = 0.2  # 0.2 > 0.1 -> FAIL
