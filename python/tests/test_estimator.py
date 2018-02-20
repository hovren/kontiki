import pytest

import numpy as np

from taser.trajectory_estimator import TrajectoryEstimator
from taser._ceres import CallbackReturnType, TerminationType

@pytest.fixture
def estimator(trajectory):
    return TrajectoryEstimator(trajectory)

def test_same_trajectory(trajectory):
    "Test that we can create estimators for all trajectory types"
    estimator = TrajectoryEstimator(trajectory)
    assert estimator.trajectory is trajectory


def test_solve_empty(estimator):
    summary = estimator.solve()
    print(summary.FullReport())
    assert summary.num_parameters == 0


def _test_add_measurement(estimator, measurements):
    for m in measurements:
        estimator.add_measurement(m)


def test_add_camera_measurement(estimator, camera_measurements):
    _test_add_measurement(estimator, camera_measurements)


def test_add_simple_measurements(estimator, simple_measurements):
    _test_add_measurement(estimator, simple_measurements)


def test_add_imu_measurements(estimator, imu_measurements):
    _test_add_measurement(estimator, imu_measurements)


def test_solve_simple_nocrash(estimator, simple_measurements):
    _test_add_measurement(estimator, simple_measurements)
    summary = estimator.solve()
    print(summary.FullReport())
    assert summary.num_parameters > 0


def test_solve_camera_nocrash(estimator, camera_measurements):
    _test_add_measurement(estimator, camera_measurements)
    summary = estimator.solve()
    print(summary.FullReport())
    assert summary.num_parameters > 0


def test_trajectory_lock(trajectory, simple_measurements):
    estimator_unlocked = TrajectoryEstimator(trajectory)
    for m in simple_measurements:
        estimator_unlocked.add_measurement(m)

    summary_unlocked = estimator_unlocked.solve()
    assert summary_unlocked.num_parameters > 0, "Measurements generated no parameters"
    print('U', summary_unlocked.num_parameters_reduced)

    estimator_locked = TrajectoryEstimator(trajectory)
    trajectory.locked = True
    for m in simple_measurements:
        estimator_locked.add_measurement(m)

    summary_locked = estimator_locked.solve()
    print('L', summary_locked.num_parameters_reduced)

    print(summary_unlocked.FullReport())
    print(summary_locked.FullReport())

    assert summary_locked.num_parameters_reduced == 0, "Not locked"


@pytest.mark.parametrize('what', ['relative_orientation', 'relative_position', 'time_offset'])
def test_imu_locks(trajectory, imu_measurements, what):
    estimator_locked = TrajectoryEstimator(trajectory)
    imus = {m.imu for m in imu_measurements}
    assert len(imus) == 1
    imu = next(i for i in imus)

    assert getattr(imu, f'{what}_locked')  # Should be locked from start

    for m in imu_measurements:
        estimator_locked.add_measurement(m)

    summary_locked = estimator_locked.solve()

    estimator_unlocked = TrajectoryEstimator(trajectory)
    setattr(imu, f'{what}_locked', False)  # Lock it
    for m in imu_measurements:
        estimator_unlocked.add_measurement(m)

    summary_unlocked = estimator_unlocked.solve()

    assert summary_unlocked.num_parameter_blocks_reduced == summary_locked.num_parameter_blocks_reduced + 1


@pytest.fixture
def callback_estimator():
    from taser.trajectories import SplitTrajectory
    from taser.measurements import PositionMeasurement
    from conftest import trajectory as gen_trajectory

    class Dummy:
        param = SplitTrajectory

    trajectory = gen_trajectory(Dummy)

    estimator = TrajectoryEstimator(trajectory)

    for t in np.linspace(*trajectory.valid_time, endpoint=False, num=20):
        m = PositionMeasurement(t, np.random.uniform(-2, 3, size=3))
        estimator.add_measurement(m)

    return estimator


def test_estimator_callback_returntype_none(callback_estimator):
    data = []
    def my_callback(iter_summary):
        data.append('Foo')

    callback_estimator.add_callback(my_callback)

    summary = callback_estimator.solve(max_iterations=4)
    print(summary.FullReport())

    assert summary.termination_type == TerminationType.Convergence


def test_estimator_callback_abort(callback_estimator):
    data = []
    def abort_immediately(iter_summary):
        data.append('Foo')
        return CallbackReturnType.Abort

    callback_estimator.add_callback(abort_immediately)

    summary = callback_estimator.solve(max_iterations=4)

    assert summary.termination_type == TerminationType.UserFailure


def test_estimator_callback_success(callback_estimator):
    data = []
    def success_immediately(iter_summary):
        return CallbackReturnType.TerminateSuccessfully

    callback_estimator.add_callback(success_immediately)

    summary = callback_estimator.solve(max_iterations=4)

    assert TerminationType.UserSuccess == summary.termination_type


def test_estimator_callback_multiple(callback_estimator):
    from collections import Counter
    class Foo:
        returned = []

        def __init__(self, x):
            self.x = x

        def callback(self, iter_summary):
            Foo.returned.append(self.x)

    foos = [Foo(i) for i in range(10)]

    for foo in foos:
        callback_estimator.add_callback(foo.callback)

    summary = callback_estimator.solve(max_iterations=5)

    counter = Counter(Foo.returned)

    # They should have been called an equal amount of times
    for i in range(1, 10):
        assert counter[i] > 1 and counter[i] == counter[0]


@pytest.mark.parametrize('update', [True, False])
def test_estimator_callback_state_update(callback_estimator, update):
    def get_knots():
        return np.vstack([knot for knot in callback_estimator.trajectory.R3_spline])

    knots0 = get_knots()
    all_knots = []

    def callback(isum):
        all_knots.append(get_knots())

    callback_estimator.add_callback(callback, update_state=update)

    callback_estimator.solve(max_iterations=5)

    if update:
        # Knots update between iterations, should be unequal
        for knots1, knots2 in zip(all_knots, all_knots[1:]):
            assert not np.allclose(knots1, knots2) and not np.allclose(knots1, 0)
    else:
        # Knots should be the same as original
        for knots in all_knots:
            np.testing.assert_equal(knots0, knots)




