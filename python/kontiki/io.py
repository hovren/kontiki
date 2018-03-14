from pathlib import Path
from contextlib import contextmanager
import warnings

import h5py
import numpy as np

from .sfm import Landmark, View
from .sensors import AtanCamera
from .trajectories import SplitTrajectory, UniformSO3SplineTrajectory, UniformR3SplineTrajectory, UniformSE3SplineTrajectory


def save_structure(fileobj, landmarks, *, group_name="structure", landmark_colors=None):
    """Save SfM structure (Views and Landmarks) in HDF5 file format

    Parameters
    ------------------
    location: str, Path, h5py.File, or h5py.Group
        If path or str, it creates a new file. If HDF5 group the data is stored below that node.
    landmarks: list of landmarks
        Landmarks to save. It will save only the views and observations that are associated with these landmarks
    group_name: str
        Name of the HDF5 group where the structure is saved.
    """
    with __create_h5_group(fileobj, group_name) as g:
        __save_structure_impl(g, landmarks, landmark_colors=landmark_colors)


def load_structure(fileobj, group_name="structure"):
    """Load SfM structure (Views and Landmarks)

    Parameters
    ------------------
    location: str, Path, h5py.File, or h5py.Group
        If path or str, it opens that HDF5 file. If HDF5 group then data is loaded from that node.
    group_name: str
        Name of the HDF5 group where the structure is stored.

    Returns
    -------
    views: list of View
        The views
    landmarks: list of Landmark
        Landmarks representing 3D poitns
    landmark_colors: dict
        Maps landmark to 8-bit RGB color
    """
    with __open_h5_group(fileobj, group_name) as g:
        return __load_structure_impl(g)


def save_trajectory(location, trajectory, group_name="trajectory"):
    """Save trajectory to HDF5 file

    Parameters
    ------------------
    location: str, Path, h5py.File, or h5py.Group
        If path or str, it creates a new file. If HDF5 group the data is stored below that node.
    trajectory: A kontiki Trajectory
        The trajectory to save
    group_name: str
        Name of the group where the trajectory is saved. If location is a path, it is a group under the root, otherwise
        it is created as a subgroup of the location.
    """
    with __create_h5_group(location, group_name) as g:
        g['type'] = trajectory.__class__.__name__
        if type(trajectory) == SplitTrajectory:
            __save_spline(g.create_group("R3_spline"), trajectory.R3_spline)
            __save_spline(g.create_group("SO3_spline"), trajectory.SO3_spline)
        else:
            __save_spline(g, trajectory)

        if type(trajectory) == UniformSE3SplineTrajectory:
            warnings.warn("SE3 trajectories currently has precison loss in the order of 10^-16 when saving to HDF5. Patches welcome!")


def load_trajectory(location, group_name="trajectory"):
    """Load trajectory from HDF5 file

    Parameters
    --------------
    location: str, Path, h5py.File, or h5py.Group
        If path or str, it opens that HDF5 file. If HDF5 group then data is loaded from that node.
    group_name: str
        The name of the group under which the trajectory is stored. If location is a path it is a group directly under the root.

    Returns
    -------------
    trajectory : A kontiki trajectory instance
    """
    with __open_h5_group(location, group_name) as g:
        trajectory_class_name = g['type'].value
        print('traj_class_name:', trajectory_class_name)
        if trajectory_class_name == 'SplitTrajectory':
            r3_spline = __load_spline(g['R3_spline'], UniformR3SplineTrajectory)
            so3_spline = __load_spline(g['SO3_spline'], UniformSO3SplineTrajectory)
            return SplitTrajectory(r3_spline, so3_spline)
        elif trajectory_class_name == 'UniformSE3SplineTrajectory':
            warnings.warn("SE3 trajectories currently has precison loss in the order of 10^-16 when saving to HDF5. Patches welcome!")
            return __load_spline(g, UniformSE3SplineTrajectory)
        elif trajectory_class_name == 'UniformSO3SplineTrajectory':
            return __load_spline(g, UniformSO3SplineTrajectory)
        elif trajectory_class_name == 'UniformR3SplineTrajectory':
            return __load_spline(g, UniformR3SplineTrajectory)


def load_atan_camera(path):
    with h5py.File(str(path)) as f:
        cols, rows = f['size'].value
        camera = AtanCamera(rows, cols,
                            f['readout'].value,
                            f['K'].value,
                            f['wc'].value,
                            f['lgamma'].value)
    return camera


@contextmanager
def __create_h5_group(location, group_name):
    try:
        # Create in existing h5py File handle
        yield location.create_group(group_name)
    except AttributeError:
        # Create file and return the group
        f = h5py.File(location, 'w')
        g = f.create_group(group_name)
        yield g
        f.close()


@contextmanager
def __open_h5_group(location, group_name):
    try:
        yield location[group_name]
    except (AttributeError, KeyError, TypeError):
        f = h5py.File(location, 'r')
        yield f[group_name]
        f.close()


def __save_structure_impl(fileobj, landmarks, landmark_colors=None):
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


def __load_structure_impl(fileobj):
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


def __save_spline(group, spline):
    """Save a spine to file or HDF5 group

    Parameters
    -------------
    location : path or h5py.Group or h5py.File
        Location to save the spline to.
    """
    group['dt'] = spline.dt
    group['t0'] = spline.t0
    knots = np.vstack([np.expand_dims(v, 0) for v in spline])

    group['knots'] = knots


def __load_spline(group, cls):
    instance = cls(group['dt'].value, group['t0'].value)
    for v in group['knots'].value:
        instance.append_knot(v)
    return instance
