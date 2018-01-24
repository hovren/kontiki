import pkgutil
import importlib

for module in pkgutil.iter_modules(__path__):
    m = importlib.import_module(f"{__package__}.{module.name}")
    traj_classes = {
        name: getattr(m, name) for name in dir(m)
        if name.endswith('Imu')
    }

    # Put the classes into this namespace
    globals().update(traj_classes)