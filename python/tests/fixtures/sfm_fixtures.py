import itertools
from pathlib import Path

import pytest
import numpy as np
import h5py

from kontiki.sfm import View, Landmark
from kontiki.io import load_structure, save_structure
from kontiki.utils import safe_time_span

def project_camera_trajectory(X_world, t0, trajectory, camera):
    def project(t):
        # World -> Trajectory
        X_traj = trajectory.from_world(X_world, t0 + t)

        # Trajectory -> Camera
        X_camera = camera.from_trajectory(X_traj)

        if X_camera[2] <= 0:
            raise ValueError("Behind camera")
        return camera.project(X_camera)

    def rootfunc(t):
        u, v = project(t)
        vt = v * camera.readout / camera.rows
        err = t - vt
        return err
    from scipy.optimize import brentq
    t = brentq(rootfunc, 0, camera.readout)
    return project(t), t0 + t


def generate_landmark(views, camera, trajectory, view_probs=None, tries=1000):
    for _ in range(tries):
        i = np.random.choice(len(views), p=view_probs)
        vi = views[i]

        # Initial observation
        u, v = np.random.uniform(0, camera.cols), np.random.uniform(0, camera.rows)
        y0 = np.array([u, v])
        z0 = np.random.uniform(0.5, 100)
        X_camera = z0 * camera.unproject(y0)
        X_trajectory = camera.to_trajectory(X_camera)
        t = vi.t0 + v * camera.readout / camera.rows
        X_world = trajectory.to_world(X_trajectory, t)

        lm = Landmark()
        lm.inverse_depth = 1 / z0
        ref = vi.create_observation(lm, y0)
        lm.reference = ref

        for v in views[i+1:]:
            try:
                (x, y), _ = project_camera_trajectory(X_world, v.t0, trajectory, camera)

                if 0 <= x < camera.cols and 0 <= y < camera.rows:
                    v.create_observation(lm, (x, y))
            except ValueError:
                pass

        if len(lm.observations) >= 2:
            return lm
        else:
            # Remove the landmark by removing its observations from the views
            for obs in lm.observations:
                obs.view.remove_observation(obs)
    raise RuntimeError("Failed to produce a valid Landmark for this trajectory and camera")


def generate_valid_structure(camera, trajectory):
    fps = 30
    nviews = 8
    t1, t2 = safe_time_span(trajectory, nviews/fps)
    # Safety margins
    t1 += 1e-2
    t2 -= 1e-2
    times = t1 + np.arange(nviews) / fps
    views = [View(i, t) for i, t in enumerate(times)]
    nlandmarks = 5
    start_probs = np.exp(-0.5*np.arange(len(views)))
    start_probs /= np.sum(start_probs)
    landmarks = [generate_landmark(views, camera, trajectory, start_probs) for _ in range(nlandmarks)]
    return views, landmarks


def save_relpose(path, relpose):
    with h5py.File(str(path), 'w') as f:
        q_ct, p_ct = relpose
        f['q_ct'] = q_ct
        f['p_ct'] = p_ct


def load_relpose(path):
    with h5py.File(str(path), 'r') as f:
        return f['q_ct'].value, f['p_ct'].value


@pytest.fixture
def small_sfm(request, camera, trajectory):
    # Load world points from cache if possible, otherwise generate it
    # Since the camera fixture randomly generates a relative pose we must store that as well
    # Note: If changes are made to the trajectory evaluation, this cache must be manually cleared!
    camera_id = camera.__class__.__name__.split('Camera')[0]
    traj_id = trajectory.__class__.__name__.split('Trajectory')[0]
    cachedir = Path(request.config.cache.makedir('structure'))
    structpath = cachedir / f'{traj_id}{camera_id}_structure.h5'
    relposepath = cachedir / f'{traj_id}{camera_id}_camera.h5'
    if not (structpath.exists() and relposepath.exists()):
        print('Generating structure and relative pose')
        views, landmarks = generate_valid_structure(camera, trajectory)
        save_structure(structpath, landmarks)
        save_relpose(relposepath, camera.relative_pose)

    # Always load fresh data from disk to ensure all tests get the same data
    print('Loading structure from cache')
    views, landmarks, _ = load_structure(structpath)
    camera.relative_pose = load_relpose(relposepath)

    return views, trajectory, camera
