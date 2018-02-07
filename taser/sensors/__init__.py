import pkgutil
import importlib

sensor_suffixes = ('Imu', 'Camera')


def is_sensor(name):
    for suffix in sensor_suffixes:
        if name.endswith(suffix):
            return True
    return False


for module in pkgutil.iter_modules(__path__):
    m = importlib.import_module(f"{__package__}.{module.name}")
    classes = {
        name: getattr(m, name) for name in dir(m)
        if is_sensor(name)
    }

    # Put the classes into this namespace
    globals().update(classes)