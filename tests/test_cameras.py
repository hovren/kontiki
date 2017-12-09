import numpy as np
import pytest
from numpy.testing import assert_almost_equal, assert_equal

from taser.cameras import AtanCamera
from taser.rotations import random_quaternion, quat_to_rotation_matrix
from fixtures.camera_fixtures import relative_pose, CAMERA_READOUT, IMAGE_ROWS, IMAGE_COLS, ATAN_K, ATAN_WC, ATAN_GAMMA


def random_image_point(camera):
    u = np.random.uniform(0, camera.cols)
    v = np.random.uniform(0, camera.rows)
    return np.array([u, v])

def test_basic(camera):
    assert camera.readout == CAMERA_READOUT
    assert camera.rows == IMAGE_ROWS
    assert camera.cols == IMAGE_COLS

    # Change and test again
    camera.readout = 0.02
    assert camera.readout == 0.02

    camera.rows = 720
    assert camera.rows == 720

    camera.cols = 1280
    assert camera.cols == 1280


def test_project_unproject(camera):
    y = random_image_point(camera)
    X = camera.unproject(y) * np.random.uniform(0.01, 10)
    yhat = camera.project(X)

    assert_almost_equal(yhat, y)


def test_pinhole(pinhole_camera):
    camera = pinhole_camera
    K = np.random.uniform(0, 20, size=(3,3))
    camera.camera_matrix = K
    assert np.allclose(camera.camera_matrix, K)


def test_atan(atan_camera):
    camera = atan_camera

    K = np.random.uniform(0, 20, size=(3,3))
    camera.camera_matrix = K
    assert np.allclose(camera.camera_matrix, K)

    wc = np.random.uniform(-1, 1, size=2)
    camera.wc = wc
    assert np.allclose(camera.wc, wc)

    gamma = np.random.uniform(0, 1)
    camera.gamma = gamma
    assert camera.gamma == gamma


def test_relative_pose_set_get(camera):
    q_ct, p_ct = relative_pose()
    camera.relative_pose = q_ct, p_ct

    qhat_ct, phat_ct = camera.relative_pose
    assert_equal(qhat_ct, q_ct)
    assert_equal(phat_ct, p_ct)


def test_relative_pose_argument_order(camera):
    q_ct, p_ct = relative_pose()
    camera.relative_pose = q_ct, p_ct # OK

    with pytest.raises(TypeError):
        camera.relative_pose = p_ct, q_ct # Error


def test_from_trajectory(camera):
    q_ct, p_ct = camera.relative_pose
    R_ct = quat_to_rotation_matrix(q_ct)

    X_trajectory = np.random.uniform(-3, 3, size=3)
    X_camera_true = R_ct @ X_trajectory + p_ct
    X_camera_actual = camera.from_trajectory(X_trajectory)
    assert_almost_equal(X_camera_actual, X_camera_true)


def test_to_trajectory(camera):
    q_ct, p_ct = camera.relative_pose
    print(q_ct, p_ct)
    R_ct = quat_to_rotation_matrix(q_ct)

    X_camera = np.random.uniform(-3, 3, size=3)
    X_trajectory_true = R_ct.T @ (X_camera - p_ct)
    X_trajectory_actual = camera.to_trajectory(X_camera)
    assert_almost_equal(X_trajectory_actual, X_trajectory_true)


# Creating the camera should not depend on which constructor is used!
def test_atan_create_unproject():
    # Using full constructor
    cam1 = AtanCamera(IMAGE_ROWS, IMAGE_COLS, CAMERA_READOUT,
                      ATAN_K, ATAN_WC, ATAN_GAMMA)

    test_project_unproject(cam1)

    # Using common constructor and setting attributes manually
    cam2 = AtanCamera(IMAGE_ROWS, IMAGE_COLS, CAMERA_READOUT)
    cam2.camera_matrix = ATAN_K
    cam2.wc = ATAN_WC
    cam2.gamma = ATAN_GAMMA
    assert_equal(cam2.camera_matrix, cam1.camera_matrix)
    assert_equal(cam2.wc, cam1.wc)
    assert cam2.gamma == cam1.gamma
    test_project_unproject(cam2)
