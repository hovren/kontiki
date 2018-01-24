import pytest

import numpy as np

from fixtures.camera_fixtures import *
from fixtures.sfm_fixtures import *

from taser.trajectories import LinearTrajectory, UniformR3SplineTrajectory, UniformSO3SplineTrajectory, SplitTrajectory
from taser.measurements import PositionMeasurement, StaticRsCameraMeasurement, GyroscopeMeasurement
from taser.sensors import BasicImu, ConstantBiasImu
from taser.utils import safe_time_span

trajectory_classes = [
    LinearTrajectory,
    UniformR3SplineTrajectory,
    UniformSO3SplineTrajectory,
    SplitTrajectory,
]

@pytest.fixture(params=trajectory_classes)
def trajectory(request):
    "Handcrafted 'simple' trajectory which is at least 5 seconds long"
    cls = request.param

    if cls == LinearTrajectory:
        t0 = 2
        k = np.array([0.1, 0, 0.4])
        return cls(t0, k)
    elif cls == UniformR3SplineTrajectory:
        dt = 2.3
        t0 = 1.22

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
    elif cls == UniformSO3SplineTrajectory:
        dt = 0.6
        t0 = 1.22

        N = int(np.ceil(5. / dt)) + 3
        times = t0 + np.arange(-3, N - 3) * dt
        w, axis = np.deg2rad(10), np.array([1., 0, 1])
        axis /= np.linalg.norm(axis)

        control_points = []
        for t in times:
            theta = w * t
            q = np.empty(4)
            q[0] = np.cos(theta / 2)
            q[1:] = np.sin(theta / 2) * axis
            control_points.append(q)

        # # Make unit quaternions and make sure they are not flipped
        # control_points /= np.linalg.norm(control_points, axis=1).reshape(-1, 1)
        # for i in range(1, len(control_points)):
        #     q1 = control_points[i-1]
        #     q2 = control_points[i]
        #     if np.dot(q1, q2) < 0:
        #         q2[:] = -q2

        instance = cls(dt, t0)
        for cp in control_points:
            instance.append_knot(cp)

        return instance
    elif cls == SplitTrajectory:
        class DummyRequest:
            def __init__(self, cls):
                self.param = cls

        # Get examples for R3 and SO3
        r3_traj = trajectory(DummyRequest(UniformR3SplineTrajectory))
        so3_traj = trajectory(DummyRequest(UniformSO3SplineTrajectory))

        instance = SplitTrajectory(r3_traj, so3_traj)
        return instance
    else:
        raise ValueError(f"Fixture simple_trajectory not available for {cls}")


imu_classes = [
    BasicImu,
    ConstantBiasImu,
]

@pytest.fixture(params=imu_classes)
def imu(request):
    cls = request.param
    return cls()

measurement_classes = [
    PositionMeasurement,
    GyroscopeMeasurement,
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


@pytest.fixture(params=[GyroscopeMeasurement])
def imu_measurements(request, imu, trajectory):
    cls = request.param
    length = 5.
    n = int(length * 3)
    times = np.linspace(*safe_time_span(trajectory, length), num=n)
    return [cls(imu, t, np.random.uniform(-1, 1, size=3)) for t in times]


@pytest.fixture(params=[PositionMeasurement])
def simple_measurements(request, trajectory):
    length = 5.
    n = int(length * 3)
    times = np.linspace(*safe_time_span(trajectory, length), num=n)
    cls = request.param
    if cls == PositionMeasurement:
        return [cls(t, np.random.uniform(-1, 1, size=3)) for t in times]
    else:
        raise NotImplementedError(f"simple_measurements fixture not implemented for {cls}")