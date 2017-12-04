import pytest

import numpy as np

from taser.cameras import PinholeCamera, AtanCamera

IMAGE_ROWS = 1080
IMAGE_COLS = 1920
CAMERA_READOUT = 0.026

ATAN_K = np.array([[853.12703455, 0., 988.06311256],
                   [0., 873.54956631, 525.71056312],
                   [0., 0., 1.]])
ATAN_WC = np.array([0.0029110778971412417, 0.0004189670467132041])#.reshape(2,1)
ATAN_GAMMA = 0.8894355177968156

@pytest.fixture
def pinhole_camera():
    K = np.eye(3)
    return PinholeCamera(IMAGE_ROWS, IMAGE_COLS, CAMERA_READOUT, K)


@pytest.fixture
def atan_camera():
    return AtanCamera(IMAGE_ROWS, IMAGE_COLS, CAMERA_READOUT,
                      ATAN_K, ATAN_WC, ATAN_GAMMA)


camera_classes = {
    PinholeCamera: pinhole_camera,
    AtanCamera: atan_camera,
}


@pytest.fixture(params=camera_classes)
def camera(request):
    cls = request.param
    try:
        return camera_classes[cls]()
    except KeyError:
        raise NotImplementedError(f"Fixture for {cls} not implemented")
