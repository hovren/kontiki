from .templatemeta import TemplateMeta
from . import _trajectory_estimator as MODULE
from . import trajectories

class Meta(TemplateMeta):
    def __call__(self, trajectory, *args, **kwargs):
        key = type(trajectory)
        try:
            cls = self._registry[key]
        except KeyError:
            raise TypeError(f"No TrajectoryEstimator declared for {type(trajectory)}")
        return cls(trajectory, *args, **kwargs)

class TrajectoryEstimator(metaclass=Meta):
    pass

estimator_classes = {
    name: getattr(MODULE, name) for name in dir(MODULE)
    if name.startswith('TrajectoryEstimator')
}

for name, impl in estimator_classes.items():
    traj_id = name.split("_")[-1]
    try:
        traj_cls = getattr(trajectories, traj_id + 'Trajectory')
        TrajectoryEstimator.register(traj_cls, impl)
    except AttributeError:
        pass