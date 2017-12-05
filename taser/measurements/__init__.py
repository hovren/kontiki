import pkgutil
import importlib

from ..templatemeta import TemplateMeta
from .. import cameras


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