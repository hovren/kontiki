import pytest

import numpy as np

from fixtures.camera_fixtures import *
from fixtures.sfm_fixtures import *

from taser.trajectories import LinearTrajectory, ConstantTrajectory, SimpleMultiTrajectory, UniformR3SplineTrajectory
from taser.measurements import PositionMeasurement, StaticRsCameraMeasurement

trajectory_classes = [
    LinearTrajectory,
    SimpleMultiTrajectory,
    UniformR3SplineTrajectory,
#    ConstantTrajectory,
]

@pytest.fixture(params=trajectory_classes)
def trajectory(request):
    "Handcrafted 'simple' trajectory"
    cls = request.param

    if cls == LinearTrajectory:
        t0 = 2
        k = np.array([0.1, 0, 0.4])
        return cls(t0, k)
    elif cls == ConstantTrajectory:
        k = np.array([1, 2, 3])
        return cls(k)
    elif cls == SimpleMultiTrajectory:
        traj = cls()
        for v in [(1, 1, 1), (10, 10, 10), (100, 100, 100)]:
            traj.foo_a.add_vector(np.array(v))
        traj.foo_b.add_vector(np.array([1, 11, 111]))
        traj.wa = 1
        traj.wb = 5

        return traj
    elif cls == UniformR3SplineTrajectory:
        dt = 0.5
        t0 = 1.0

        control_points = [
            np.array([1, 1, 2]),
            np.array([1, 2, 1.4]),
            np.array([1, 4, 0]),
            np.array([-2, 2, 2]),
            np.array([-3, -2, 1]),
            np.array([-4, -2, 0])
        ]

        instance = cls(dt, t0)

        for cp in control_points:
            instance.append_knot(cp)
        return instance
    else:
        raise ValueError(f"Fixture simple_trajectory not available for {cls}")


measurement_classes = [
    PositionMeasurement,
    StaticRsCameraMeasurement,
]

@pytest.fixture(params=[StaticRsCameraMeasurement])
def camera_measurements(request, small_sfm):
    views, trajectory, camera = small_sfm
    MeasurementClass = request.param

    landmarks = {obs.landmark for v in views for obs in v.observations}

    measurements = []
    for lm in landmarks:
        for obs in lm.observations:
            if obs is not lm.reference:
                m = MeasurementClass(camera, obs)
                measurements.append(m)
    return measurements

@pytest.fixture(params=[PositionMeasurement])
def simple_measurements(request):
    times = np.linspace(0, 2, num=10)
    cls = request.param
    if cls == PositionMeasurement:
        return [cls(t, np.random.uniform(-1, 1, size=3)) for t in times]