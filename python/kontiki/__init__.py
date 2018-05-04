from .templatemeta import TemplateMeta
from . import _trajectory_estimator as __MODULE
from . import trajectories

# Load version from compiled module
from ._version import __version__

# Import Ceres-Solver classes into kontiki namespace
from ._ceres import Summary, IterationSummary, CallbackReturnType

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
    name: getattr(__MODULE, name) for name in dir(__MODULE)
    if name.startswith('TrajectoryEstimator')
}

for name, impl in estimator_classes.items():
    traj_id = name.split("_")[-1]
    try:
        traj_cls = getattr(trajectories, traj_id)
        TrajectoryEstimator.register(traj_cls, impl)
    except AttributeError:
        pass


# import inspect
# p = inspect.Parameter('measurement', inspect.Parameter.POSITIONAL_OR_KEYWORD, annotation=int)
# sig = inspect.Signature([p])
# TrajectoryEstimator.add_measurement.__func__.__signature__ = sig
# TrajectoryEstimator.add_measurement.__func__.__doc__ = "New doc string"