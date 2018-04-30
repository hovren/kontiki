import numpy as np
import pytest
from numpy.testing import assert_almost_equal, assert_equal
import scipy.misc

from kontiki.sensors import AtanCamera
from kontiki.rotations import random_quaternion, quat_to_rotation_matrix
from fixtures.camera_fixtures import CAMERA_READOUT, IMAGE_ROWS, IMAGE_COLS, ATAN_K, ATAN_WC, ATAN_GAMMA


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


def test_derivative(camera):
    y = random_image_point(camera)
    X = camera.unproject(y) * np.random.uniform(3, 10)

    # Random derivative of the landmark
    dX = X + np.random.normal(size=3)

    # Analytical differentiation
    _, dy = camera.evaluate_projection(X, dX, True)

    # Numerical differentiation
    # dy = Jf(X) @ Jx  where Jf(X) is the jacobian of f at the point X and Jx=dx is the jacobian of X
    # Start by constructing Jf(X) by differentiating each element
    def f_jacobian_element(fi, xi):
        X0 = np.copy(X)
        def func(x):
            X0[xi] = x
            f = camera.project(X0)
            return f[fi]
        return scipy.misc.derivative(func, X0[xi], dx=1e-3)

    f_jac = np.empty((2, 3))
    for fi, xi in np.ndindex(f_jac.shape):
        f_jac[fi, xi] = f_jacobian_element(fi, xi)

    # Construct the numerical derivative, and compare it
    dy_num = f_jac @ dX
    np.testing.assert_almost_equal(dy_num, dy, decimal=3)

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
