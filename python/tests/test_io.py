import pytest
import numpy as np
from numpy.testing import assert_equal, assert_almost_equal
import h5py

from kontiki.sfm import Landmark, View
from kontiki.io import save_structure, load_structure, save_trajectory, load_trajectory
from kontiki.trajectories import UniformSE3SplineTrajectory, UniformSO3SplineTrajectory, UniformR3SplineTrajectory, SplitTrajectory, LinearTrajectory

def random_structure():
    frame_times = np.arange(0, np.random.uniform(1.5, 3), np.random.uniform(1/30, 1/10))
    views = [View(n, t) for n, t in enumerate(frame_times)]
    landmarks = [Landmark() for _ in range(np.random.randint(10, 20))]
    landmark_colors = {lm: np.random.randint(0, 255, 3) for lm in landmarks}

    for v in views:
        n = np.random.randint(1, len(landmarks)-1)
        for lm in np.random.choice(landmarks, n, replace=False):
            uv = np.random.uniform(0, 1000, size=2)
            v.create_observation(lm, uv)

    for lm in landmarks:
        lm.reference = lm.observations[0]
    return views, landmarks, landmark_colors


def assert_structure_equal(landmarks1, colors1, landmarks2, colors2):
    orig_views = list({obs.view for lm in landmarks1 for obs in lm.observations})
    orig_views.sort(key=lambda v: v.frame_nr)
    loaded_views = list({obs.view for lm in landmarks2 for obs in lm.observations})
    loaded_views.sort(key=lambda v: v.frame_nr)

    assert len(orig_views) == len(loaded_views), "Number of views differ"
    view_map = {}
    for v1, v2 in zip(orig_views, loaded_views):
        assert v1.frame_nr == v2.frame_nr, "Frame numbers differ"
        assert v1.t0 == v2.t0, "t0 differs"
        view_map[v2] = v1

    # Landmarks are meant to be loaded in the same order they are saved
    assert len(landmarks1) == len(landmarks2), "Number of landmarks differ"
    for lm_orig, lm_load in zip(landmarks1, landmarks2):
        assert len(lm_orig.observations) == len(lm_load.observations), "Number of observations differ"
        obs_map = {obs_load: obs_orig for obs_orig, obs_load in zip(lm_orig.observations, lm_load.observations)}

        assert lm_orig.reference is obs_map[lm_load.reference], "Reference observations is different"
        for obs_load, obs_orig in obs_map.items():
            assert_equal(obs_orig.uv, obs_load.uv)
            assert obs_orig.view is view_map[obs_load.view], "Views differ"

        assert_equal(colors1[lm_orig], colors2[lm_load], err_msg="Colors are different")



def test_save_load_structure(tmpdir):
    views, landmarks, landmark_colors = random_structure()

    f = tmpdir.mkdir("out").join("structure_save.h5")

    # Save subset of landmarks
    num_to_save = int(0.75 * len(landmarks))
    selected_landmarks = np.random.choice(landmarks, num_to_save, replace=False)
    save_structure(str(f), selected_landmarks, landmark_colors=landmark_colors)

    # Load again
    loaded_views, loaded_landmarks, loaded_colors = load_structure(str(f))

    assert_structure_equal(selected_landmarks, landmark_colors, loaded_landmarks, loaded_colors)


def test_save_load_structure_grouped(tmpdir):
    views1, landmarks1, colors1 = random_structure()
    views2, landmarks2, colors2 = random_structure()

    selected1 = np.random.choice(landmarks1, int(0.75 * len(landmarks1)), replace=False)
    selected2 = np.random.choice(landmarks2, int(0.75 * len(landmarks2)), replace=False)

    filename = tmpdir.mkdir("out").join("structure_save_grouped.h5")

    with h5py.File(filename, 'w') as f:
        print(f'Saving {len(selected1)} landmarks in first')
        save_structure(f, selected1, group_name="first", landmark_colors=colors1)
        print(f'Saving {len(selected2)} landmarks in second')
        save_structure(f, selected2, group_name="second", landmark_colors=colors2)

    with h5py.File(filename, 'r') as f:
        loaded_views1, loaded_landmarks1, loaded_colors1 = load_structure(f, group_name="first")
        loaded_views2, loaded_landmarks2, loaded_colors2 = load_structure(f, group_name="second")

    assert_structure_equal(selected1, colors1, loaded_landmarks1, loaded_colors1)
    assert_structure_equal(selected2, colors2, loaded_landmarks2, loaded_colors2)


def assert_spline_equal(spline1, spline2):
    assert type(spline1) == type(spline2), "Splines had different types"
    assert spline1.t0 == spline2.t0, f"t0: {spline1.t0} != {spline2.t0}"
    assert spline1.dt == spline2.dt, f"dt: {spline1.dt} != {spline2.dt}"
    assert len(spline1) == len(spline2), f"Length: {len(spline1)} != {len(spline2)}"

    for i, (v1, v2) in enumerate(zip(spline1, spline2)):
        #assert_equal(v1, v2, err_msg=f"knot {i} did not match")
        assert_almost_equal(v1, v2, decimal=15)


def assert_trajectory_equal(traj1, traj2):
    assert type(traj1) == type(traj2), "Trajectories had different types"
    if type(traj1) == SplitTrajectory:
        assert_spline_equal(traj1.SO3_spline, traj2.SO3_spline)
        assert_spline_equal(traj1.R3_spline, traj2.R3_spline)
    else:
        assert_spline_equal(traj1, traj2)


def test_save_load_trajectory(tmpdir, trajectory):
    if type(trajectory) == LinearTrajectory:
        pytest.xfail("We have not implemented I/O for LinearTrajectory")

    f = tmpdir.join("trajectory.h5")

    # SE3 trajectories have a weird bug where there is some precision loss (in the order of 10^-16) while saving to HDF5
    if type(trajectory) == UniformSE3SplineTrajectory:
        with pytest.warns(Warning):
            save_trajectory(str(f), trajectory)
        with pytest.warns(Warning):
            loaded = load_trajectory(str(f))
    else:
        save_trajectory(str(f), trajectory)
        loaded = load_trajectory(str(f))

    assert_trajectory_equal(loaded, trajectory)


def test_save_load_trajectory_grouped(tmpdir, trajectory):
    if type(trajectory) == LinearTrajectory:
        pytest.xfail("We have not implemented I/O for LinearTrajectory")

    # Create a new trajectory that is slightly longer by extending the splines
    trajectory2 = trajectory.clone()

    if type(trajectory) == SplitTrajectory:
        splines_to_extend = [trajectory2.SO3_spline, trajectory2.R3_spline]
    else:
        splines_to_extend = [trajectory]

    for spline in splines_to_extend:
        spline.extend_to(spline.max_time + 10 * spline.dt, spline[-1])

    filename = tmpdir.join("trajectory_multi.h5")
    with h5py.File(filename, 'w') as f:
        save_trajectory(f, trajectory, group_name="first")
        save_trajectory(f, trajectory2, group_name="second")

    with h5py.File(filename, 'r') as f:
        loaded = load_trajectory(f, group_name="first")
        loaded2 = load_trajectory(f, group_name="second")

    assert_trajectory_equal(loaded, trajectory)
    assert_trajectory_equal(loaded2, trajectory2)
