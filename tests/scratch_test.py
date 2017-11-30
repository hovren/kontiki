import pytest

import numpy as np

from taser.trajectories._constant_trajectory import ConstantTrajectory
from taser.measurements._position_measurement import PositionMeasurement
from taser.measurements._position_measurement import AnotherMeasurement
from taser._trajectory_estimator import TrajectoryEstimator

def test_scratch():
    constant = np.array([1, 2, 3])
    trajectory = ConstantTrajectory(constant)
    t = 4.5
    p = np.array([12, 3, 8])
    meas = PositionMeasurement(t, p)

    m2 = AnotherMeasurement(t, p)

    estimator = TrajectoryEstimator(trajectory)
    estimator.add_measurement(meas)
    estimator.add_measurement(m2)

    summary = estimator.solve()
    print(summary.BriefReport())
    np.testing.assert_almost_equal(trajectory.constant, p)