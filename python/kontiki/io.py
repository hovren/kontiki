from pathlib import Path

import h5py
import numpy as np

from .sfm import Landmark, View
from .sensors import AtanCamera


def save_structure(fileobj, landmarks, *, landmark_colors=None):
    "Save SfM structure (Views and Landmarks)"
    if isinstance(fileobj, h5py.File) or isinstance(fileobj, h5py.Group):
        _save_structure_impl(fileobj, landmarks, landmark_colors)
    else:
        with h5py.File(fileobj, 'w') as f:
            _save_structure_impl(f, landmarks, landmark_colors)


def load_structure(fileobj):
    """Load SfM structure (Views and Landmarks)

    Parameters
    -----------
    fileobj : h5py.File, h5py.Group, str
        Either a path to a file, or an h5py file-like object.

    Returns
    -------
    views: list of View
        The views
    landmarks: list of Landmark
        Landmarks representing 3D poitns
    landmark_colors: dict
        Maps landmark to 8-bit RGB color
    """
    if isinstance(fileobj, h5py.File) or isinstance(fileobj, h5py.Group):
        return _load_structure_impl(fileobj)
    else:
        with h5py.File(fileobj, 'r') as f:
            return _load_structure_impl(f)


def _save_structure_impl(fileobj, landmarks, landmark_colors=None):
    groot = fileobj.create_group('structure')

    views = list({obs.view for lm in landmarks for obs in lm.observations})
    views.sort(key=lambda v: v.frame_nr)

    def observations(): # Loop over all observations
        for lm in landmarks:
            for obs in lm.observations:
                yield obs

    view_to_index = {v: i for i, v in enumerate(views)}
    landmark_to_index = {lm: i for i, lm in enumerate(landmarks)}
    obs_to_index = {obs: i for i, obs in enumerate(observations())}

    gviews = groot.create_group('views')
    gviews['frame_nr'] = np.array([v.frame_nr for v in views], dtype='int')
    gviews['t0'] = np.array([v.t0 for v in views])

    glandmarks = groot.create_group('landmarks')
    glandmarks['inverse_depth'] = np.array([lm.inverse_depth for lm in landmarks])
    glandmarks['ref_idx'] = np.array([obs_to_index[lm.reference] for lm in landmarks], dtype='int')

    gobs = groot.create_group('observations')
    gobs['uv'] = np.vstack([obs.uv for obs in observations()])
    gobs['lm_idx'] = np.array([landmark_to_index[obs.landmark] for obs in observations()], dtype='int')
    gobs['v_idx'] = np.array([view_to_index[obs.view] for obs in observations()], dtype='int')

    if landmark_colors:
        print('Saving colors')
        colors = np.vstack([landmark_colors[lm] for lm in landmarks])
    else:
        print('No colors to save')
        colors = np.empty((0,3))
    glandmarks['color'] = colors


def _load_structure_impl(fileobj):
    groot = fileobj['structure']
    gviews = groot['views']

    views = [View(fnr, t0) for fnr, t0 in zip(gviews['frame_nr'].value, gviews['t0'])]

    glandmarks = groot['landmarks']
    landmarks = [Landmark() for _ in range(len(glandmarks['inverse_depth']))]

    gobs = groot['observations']
    observations = []
    for uv, lm_idx, v_idx in zip(gobs['uv'].value, gobs['lm_idx'].value, gobs['v_idx'].value):
        view = views[v_idx]
        lm = landmarks[lm_idx]
        obs = view.create_observation(lm, uv)
        observations.append(obs)

    for lm, invd, ref_idx in zip(landmarks, glandmarks['inverse_depth'].value, glandmarks['ref_idx'].value):
        lm.inverse_depth = invd
        lm.reference = observations[ref_idx]

    colors = glandmarks['color'].value
    if len(colors) == len(landmarks):
        print('Loading colors')
        landmark_colors = {lm: c for lm, c in zip(landmarks, colors)}
    elif len(colors) == 0:
        print('No colors')
        landmark_colors = None
    else:
        raise IOError("Number of colors do not match!")

    return views, landmarks, landmark_colors
    

def load_atan_camera(path):
    with h5py.File(str(path)) as f:
        cols, rows = f['size'].value
        camera = AtanCamera(rows, cols,
                            f['readout'].value,
                            f['K'].value,
                            f['wc'].value,
                            f['lgamma'].value)
    return camera    

