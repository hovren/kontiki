import itertools
from pathlib import Path

import pytest
import numpy as np

from taser.sfm import View, Landmark
from taser.io import load_structure, save_structure

def project_camera_trajectory(X_world, t0, trajectory, camera):
    def project(t):
        # World -> Trajectory
        X_traj = trajectory.from_world(X_world, t0 + t)

        # Trajectory -> Camera
        X_camera = X_traj

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


def generate_landmark(views, camera, trajectory, view_probs=None, tries=10):
    for _ in range(tries):
        i = np.random.choice(len(views), p=view_probs)
        vi = views[i]

        # Initial observation
        u, v = np.random.uniform(0, camera.cols), np.random.uniform(0, camera.rows)
        y0 = np.array([u, v])
        z0 = np.random.uniform(0.5, 100)
        X_camera = z0 * camera.unproject(y0)
        X_trajectory = X_camera
        t = vi.t0 + v * camera.readout / camera.rows
        X_world = trajectory.to_world(X_trajectory, t)

        lm = Landmark()
        lm.inverse_depth = 1 / z0
        ref = vi.create_observation(lm, u, v)
        lm.reference = ref

        for v in views[i+1:]:
            try:
                (x, y), _ = project_camera_trajectory(X_world, v.t0, trajectory, camera)

                if 0 <= x < camera.cols and 0 <= y < camera.rows:
                    v.create_observation(lm, x, y)
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
    views = [View(i, i/fps) for i in range(nviews)]
    nlandmarks = 5
    start_probs = np.exp(-0.5*np.arange(len(views)))
    start_probs /= np.sum(start_probs)
    landmarks = [generate_landmark(views, camera, trajectory, start_probs)]
    return views, landmarks

@pytest.fixture
def small_sfm(request, camera, trajectory):
    # Load world points from cache if possible, otherwise generate it
    camera_id = camera.__class__.__name__.split('Camera')[0]
    traj_id = trajectory.__class__.__name__.split('Trajectory')[0]
    cachedir = Path(request.config.cache.makedir('structure'))
    cachepath = cachedir / f'{traj_id}{camera_id}_structure.h5'
    if not cachepath.exists():
        print('Generating structure')
        views, landmarks = generate_valid_structure(camera, trajectory)
        save_structure(cachepath, landmarks)

    # Always load fresh data from disk to ensure all tests get the same
    print('Loading structure from cache')
    views, landmarks, _ = load_structure(cachepath)


    return views, trajectory, camera


