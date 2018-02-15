import pytest
import numpy as np
from numpy.testing import assert_equal

from taser.sfm import Landmark, View
from taser.io import save_structure, load_structure

def test_structure(tmpdir):
    frame_times = np.arange(0, 1.5, 1/30)
    views = [View(n, t) for n, t in enumerate(frame_times)]
    landmarks = [Landmark() for _ in range(50)]
    landmark_colors = {lm: np.random.randint(0, 255, 3) for lm in landmarks}

    for v in views:
        n = np.random.randint(1, len(landmarks)-1)
        for lm in np.random.choice(landmarks, n, replace=False):
            uv = np.random.uniform(0, 1000, size=2)
            v.create_observation(lm, *uv)

    for lm in landmarks:
        lm.reference = lm.observations[0]

    f = tmpdir.mkdir("out").join("structure_save.h5")

    # Save subset of landmarks
    selected_landmarks = np.random.choice(landmarks, 30, replace=False)
    save_structure(str(f), selected_landmarks, landmark_colors=landmark_colors)

    # Load again
    loaded_views, loaded_landmarks, loaded_colors = load_structure(str(f))
    orig_views = list({obs.view for lm in selected_landmarks for obs in lm.observations})
    orig_views.sort(key=lambda v: v.frame_nr)
    loaded_views = list({obs.view for lm in loaded_landmarks for obs in lm.observations})
    loaded_views.sort(key=lambda v: v.frame_nr)

    assert len(orig_views) == len(loaded_views)
    view_map = {}
    for v1, v2 in zip(orig_views, loaded_views):
        assert v1.frame_nr == v2.frame_nr
        assert v1.t0 == v2.t0
        view_map[v2] = v1

    # Landmarks are meant to be loaded in the same order they are saved
    assert len(selected_landmarks) == len(loaded_landmarks)
    for lm_orig, lm_load in zip(selected_landmarks, loaded_landmarks):
        assert len(lm_orig.observations) == len(lm_load.observations)
        obs_map = {obs_load: obs_orig for obs_orig, obs_load in zip(lm_orig.observations, lm_load.observations)}

        assert lm_orig.reference is obs_map[lm_load.reference]
        for obs_load, obs_orig in obs_map.items():
            assert_equal(obs_orig.uv, obs_load.uv)
            assert obs_orig.view is view_map[obs_load.view]

        assert_equal(landmark_colors[lm_orig], loaded_colors[lm_load])
