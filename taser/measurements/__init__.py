import pkgutil
import importlib
import inspect
import collections

from ..templatemeta import TemplateMeta
from .. import sensors


for module in pkgutil.iter_modules(__path__):
    m = importlib.import_module(f"{__package__}.{module.name}")
    classes = {
        name: getattr(m, name) for name in dir(m)
        if name.endswith('Measurement')
    }

    # Put the classes into this namespace
    globals().update(classes)


class RsMeta(TemplateMeta):
    def __call__(self, camera, *args, **kwargs):
        key = type(camera)
        try:
            cls = self._registry[key]
        except KeyError:
            raise TypeError(f"No class declared for {type(camera)}")
        return cls(camera, *args, **kwargs)

class StaticRsCameraMeasurement(metaclass=RsMeta):
    pass


class LiftingRsCameraMeasurement(metaclass=RsMeta):
    pass


class ImuMeta(TemplateMeta):
    def __call__(self, imu, *args, **kwargs):
        key = type(imu)
        try:
            cls = self._registry[key]
        except KeyError:
            raise TypeError(f"No class declared for {type(imu)}")
        return cls(imu, *args, **kwargs)


class GyroscopeMeasurement(metaclass=ImuMeta):
    pass


class AccelerometerMeasurement(metaclass=ImuMeta):
    pass


# Gather all measurement implementations
class_implementations = collections.defaultdict(list)
for module in pkgutil.iter_modules(__path__):
    m = importlib.import_module(f"{__package__}.{module.name}")

    for name in dir(m):
        attr = getattr(m, name)
        if inspect.isclass(attr):
            root, *_ = name.split("_")
            if root.endswith('Measurement'):
                class_implementations[root].append(attr)

# Register with python classes
for base_name, impl_classes in class_implementations.items():
    reg_cls = globals()[base_name]

    if hasattr(reg_cls, 'register'):
        for impl_cls in impl_classes:
            _, *impl_sensors_ids = impl_cls.__name__.split("_")
            impl_sensors = [getattr(sensors, class_id) for class_id in impl_sensors_ids]
            key = tuple(impl_sensors) if len(impl_sensors) > 1 else impl_sensors[0]
            reg_cls.register(key, impl_cls)