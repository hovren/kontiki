import pkgutil
import importlib

from ..templatemeta import TemplateMeta
from .. import cameras, sensors


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


from . import _static_rscamera_measurement as STATIC_MODULE
static_classes = {
    name: getattr(STATIC_MODULE, name) for name in dir(STATIC_MODULE)
    if name.startswith('StaticRsCameraMeasurement_')
}

for name, impl in static_classes.items():
    camera_id = name.split("_")[-1]
    try:
        camera_cls = getattr(cameras, camera_id + 'Camera')
        StaticRsCameraMeasurement.register(camera_cls, impl)
    except AttributeError:
        pass

from . import _gyroscope_measurement as GYROSCOPE_MODULE
gyroscope_classes = {
    name: getattr(GYROSCOPE_MODULE, name) for name in dir(GYROSCOPE_MODULE)
    if name.startswith('GyroscopeMeasurement_')
}

for name, impl in gyroscope_classes.items():
    imu_id = name.split("_")[-1]
    try:
        imu_cls = getattr(sensors, imu_id)
        GyroscopeMeasurement.register(imu_cls, impl)
    except AttributeError:
        pass


from . import _accelerometer_measurement as ACCELEROMETER_MODULE
accelerometer_classes = {
    name: getattr(ACCELEROMETER_MODULE, name) for name in dir(ACCELEROMETER_MODULE)
    if name.startswith('AccelerometerMeasurement_')
}

for name, impl in accelerometer_classes.items():
    imu_id = name.split("_")[-1]
    try:
        imu_cls = getattr(sensors, imu_id)
        AccelerometerMeasurement.register(imu_cls, impl)
    except AttributeError:
        pass