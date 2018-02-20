import pkgutil
import importlib
import numpy as np

from ..templatemeta import continueClass

for module in pkgutil.iter_modules(__path__):
    m = importlib.import_module(f"{__package__}.{module.name}")
    traj_classes = {
        name: getattr(m, name) for name in dir(m)
        if name.endswith('Trajectory')
    }

    # Put the classes into this namespace
    globals().update(traj_classes)
