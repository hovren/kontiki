import pkgutil
import importlib

for module in pkgutil.iter_modules(__path__):
    m = importlib.import_module(f"{__package__}.{module.name}")
    classes = {
        name: getattr(m, name) for name in dir(m)
        if name.endswith('Measurement')
    }

    # Put the classes into this namespace
    globals().update(classes)