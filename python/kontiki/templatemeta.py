# Copyright 2008-2017  AURA/LSST.
#
# This product includes software developed by the
# LSST Project (http://www.lsst.org/).
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the LSST License Statement and
# the GNU General Public License along with this program.  If not,
# see <https://www.lsstcorp.org/LegalNotices/>.

import sys

INTRINSIC_SPECIAL_ATTRIBUTES = frozenset((
    "__qualname__",
    "__module__",
    "__metaclass__",
    "__dict__",
    "__weakref__",
    "__class__",
    "__subclasshook__",
    "__name__",
    "__doc__",
))

def isAttributeSafeToTransfer(name, value):
    """Return True if an attribute is safe to monkeypatch-transfer to another
    class.
    This rejects special methods that are defined automatically for all
    classes, leaving only those explicitly defined in a class decorated by
    `continueClass` or registered with an instance of `TemplateMeta`.
    """
    if name.startswith("__") and (value is getattr(object, name, None) or
                                          name in INTRINSIC_SPECIAL_ATTRIBUTES):
        return False
    return True


def continueClass(cls):
    """Re-open the decorated class, adding any new definitions into the original.
    For example,
    ::
        class Foo:
            pass
        @continueClass
        class Foo:
            def run(self):
                return None
    is equivalent to
    ::
        class Foo:
            def run(self):
                return None
    """
    orig = getattr(sys.modules[cls.__module__], cls.__name__)
    for name in dir(cls):
        # Common descriptors like classmethod and staticmethod can only be
        # accessed without invoking their magic if we use __dict__; if we use
        # getattr on those we'll get e.g. a bound method instance on the dummy
        # class rather than a classmethod instance we can put on the target
        # class.
        attr = cls.__dict__.get(name, None) or getattr(cls, name)
        if isAttributeSafeToTransfer(name, attr):
            setattr(orig, name, attr)
    return orig


def inClass(cls, name=None):
    """Add the decorated function to the given class as a method.
    For example,
    ::
        class Foo:
            pass
        @inClass(Foo)
        def run(self):
            return None
    is equivalent to::
        class Foo:
            def run(self):
                return None
    Standard decorators like ``classmethod``, ``staticmethod``, and
    ``property`` may be used *after* this decorator.  Custom decorators
    may only be used if they return an object with a ``__name__`` attribute
    or the ``name`` optional argument is provided.
    """
    def decorate(func):
        # Using 'name' instead of 'name1' breaks the closure because
        # assignment signals a strictly local variable.
        name1 = name
        if name1 is None:
            if hasattr(func, "__name__"):
                name1 = func.__name__
            else:
                if hasattr(func, "__func__"):
                    # classmethod and staticmethod have __func__ but no __name__
                    name1 = func.__func__.__name__
                elif hasattr(func, "fget"):
                    # property has fget but no __name__
                    name1 = func.fget.__name__
                else:
                    raise ValueError(
                        "Could not guess attribute name for '{}'.".format(func)
                    )
        setattr(cls, name1, func)
        return func
    return decorate

class TemplateMeta(type):
    """A metaclass for abstract base classes that tie together wrapped C++
    template types.
    C++ template classes are most easily wrapped with a separate Python class
    for each template type, which results in an unnatural Python interface.
    TemplateMeta provides a thin layer that connects these Python classes by
    giving them a common base class and acting as a factory to construct them
    in a consistent way.
    To use, simply create a new class with the name of the template class, and
    use ``TemplateMeta`` as its metaclass, and then call ``register`` on each
    of its subclasses.  This registers the class with a "type key" - usually a
    Python representation of the C++ template types.  The type key must be a
    hashable object - strings, type objects, and tuples of these (for C++
    classes with multiple template parameters) are good choices.  Alternate
    type keys for existing classes can be added by calling ``alias``, but only
    after a subclass already been registered with a "primary" type key.  For
    example (using Python 3 metaclass syntax)::
        import numpy as np
        from ._image import ImageF, ImageD
        class Image(metaclass=TemplateMeta):
            pass
        Image.register(np.float32, ImageF)
        Image.register(np.float64, ImageD)
        Image.alias("F", ImageF)
        Image.alias("D", ImageD)
    We have intentionally used ``numpy`` types as the primary keys for these
    objects in this example, with strings as secondary aliases simply because
    the primary key is added as a ``dtype`` attribute on the the registered
    classes (so ``ImageF.dtype == numpy.float32`` in the above example).
    This allows user code to construct objects directly using ``Image``, as
    long as an extra ``dtype`` keyword argument is passed that matches one of
    the type keys::
        img = Image(52, 64, dtype=np.float32)
    This simply forwards additional positional and keyword arguments to the
    wrapped template class's constructor.
    The choice of "dtype" as the name of the template parameter is also
    configurable, and in fact multiple template parameters are also supported,
    by setting a ``TEMPLATE_PARAMS`` class attribute on the ABC to a tuple
    containing the names of the template parameters.  A ``TEMPLATE_DEFAULTS``
    attribute can also be defined to a tuple of the same length containing
    default values for the template parameters, allowing them to be omitted in
    constructor calls.  When the length of these attributes is more than one,
    the type keys passed to ``register`` and ``alias`` should be tuple of the
    same length; when the length of these attributes is one, type keys should
    generally not be tuples.
    As an aid for those writing the Python wrappers for C++ classes,
    ``TemplateMeta`` also provides a way to add pure-Python methods and other
    attributes to the wrapped template classes.  To add a ``sum`` method to
    all registered types, for example, we can just do::
        class Image(metaclass=TemplateMeta):
            def sum(self):
                return np.sum(self.getArray())
        Image.register(np.float32, ImageF)
        Image.register(np.float64, ImageD)
    .. note::
        ``TemplateMeta`` works by overriding the ``__instancecheck__`` and
        ``__subclasscheck__`` special methods, and hence does not appear in
        its registered subclasses' method resolution order or ``__bases__``
        attributes. That means its attributes are not inherited by registered
        subclasses. Instead, attributes added to an instance of
        ``TemplateMeta`` are *copied* into the types registered with it. These
        attributes will thus *replace* existing attributes in those classes
        with the same name, and subclasses cannot delegate to base class
        implementations of these methods.
    Finally, abstract base classes that use ``TemplateMeta`` define a dict-
    like interface for accessing their registered subclasses, providing
    something like the C++ syntax for templates::
        Image[np.float32] -> ImageF
        Image["D"] -> ImageD
    Both primary dtypes and aliases can be used as keys in this interface,
    which means types with aliases will be present multiple times in the dict.
    To obtain the sequence of unique subclasses, use the ``__subclasses__``
    method.
    """

    def __new__(cls, name, bases, attrs):
        # __new__ is invoked when the abstract base class is defined (via a
        # class statement).  We save a dict of class attributes (including
        # methods) that were defined in the class body so we can copy them
        # to registered subclasses later.
        # We also initialize an empty dict to store the registered subclasses.
        attrs["_inherited"] = {k: v for k, v in attrs.items()
                               if isAttributeSafeToTransfer(k, v)}
        # The special "TEMPLATE_PARAMS" class attribute, if defined, contains
        # names of the template parameters, which we use to set those
        # attributes on registered subclasses and intercept arguments to the
        # constructor.  This line removes it from the dict of things that
        # should be inherited while setting a default of 'dtype' if it's not
        # defined.
        attrs["TEMPLATE_PARAMS"] = \
            attrs["_inherited"].pop("TEMPLATE_PARAMS", ("dtype",))
        attrs["TEMPLATE_DEFAULTS"] = \
            attrs["_inherited"].pop("TEMPLATE_DEFAULTS",
                                    (None,)*len(attrs["TEMPLATE_PARAMS"]))
        attrs["_registry"] = dict()
        self = type.__new__(cls, name, bases, attrs)

        if len(self.TEMPLATE_PARAMS) == 0:
            raise ValueError(
                "TEMPLATE_PARAMS must be a tuple with at least one element."
            )
        if len(self.TEMPLATE_DEFAULTS) != len(self.TEMPLATE_PARAMS):
            raise ValueError(
                "TEMPLATE_PARAMS and TEMPLATE_DEFAULTS must have same length."
            )
        return self

    def __call__(self, *args, **kwds):
        # __call__ is invoked when someone tries to construct an instance of
        # the abstract base class.
        # If the ABC defines a "TEMPLATE_PARAMS" attribute, we use those strings
        # as the kwargs we should intercept to find the right type.
        key = tuple(kwds.pop(p, d) for p, d in zip(self.TEMPLATE_PARAMS,
                                                   self.TEMPLATE_DEFAULTS))
        # indices are only tuples if there are multiple elements
        cls = self._registry.get(key[0] if len(key) == 1 else key, None)
        if cls is None:
            d = {k: v for k, v in zip(self.TEMPLATE_PARAMS, key)}
            raise TypeError("No registered subclass for {}.".format(d))
        return cls(*args, **kwds)

    def __subclasscheck__(self, subclass):
        # Special method hook for the issubclass built-in: we return true for
        # any registered type or true subclass thereof.
        if subclass in self._registry:
            return True
        for v in self._registry.values():
            if issubclass(subclass, v):
                return True
        return False

    def __instancecheck__(self, instance):
        # Special method hook for the isinstance built-in: we return true for
        # an instance of any registered type or true subclass thereof.
        if type(instance) in self._registry:
            return True
        for v in self._registry.values():
            if isinstance(instance, v):
                return True
        return False

    def __subclasses__(self):
        """Return a tuple of all classes that inherit from this class.
        """
        # This special method isn't defined as part of the Python data model,
        # but it exists on builtins (including ABCMeta), and it provides useful
        # functionality.
        return tuple(set(self._registry.values()))

    def register(self, key, subclass):
        """Register a subclass of this ABC with the given key (a string,
        number, type, or other hashable).
        Register may only be called once for a given key or a given subclass.
        """
        if key is None:
            raise ValueError("None may not be used as a key.")
        if subclass in self._registry.values():
            raise ValueError(
                "This subclass has already registered with another key; "
                "use alias() instead."
            )
        if self._registry.setdefault(key, subclass) != subclass:
            if len(self.TEMPLATE_PARAMS) == 1:
                d = {self.TEMPLATE_PARAMS[0]: key}
            else:
                d = {k: v for k, v in zip(self.TEMPLATE_PARAMS, key)}
            raise KeyError(
                "Another subclass is already registered with {}".format(d)
            )

        def setattrSafe(name, value):
            try:
                currentValue = getattr(subclass, name)
                if currentValue != value:
                    msg = ("subclass already has a '{}' attribute with "
                           "value {} != {}.")
                    raise ValueError(
                        msg.format(name, currentValue, value)
                    )
            except AttributeError:
                setattr(subclass, name, value)

        if len(self.TEMPLATE_PARAMS) == 1:
            setattrSafe(self.TEMPLATE_PARAMS[0], key)
        elif len(self.TEMPLATE_PARAMS) == len(key):
            for p, k in zip(self.TEMPLATE_PARAMS, key):
                setattrSafe(p, k)
        else:
            raise ValueError(
                "key must have {} elements (one for each of {})".format(
                    len(self.TEMPLATE_PARAMS), self.TEMPLATE_PARAMS
                )
            )

        for name, attr in self._inherited.items():
            setattr(subclass, name, attr)

    def alias(self, key, subclass):
        """Add an alias that allows an existing subclass to be accessed with a
        different key.
        """
        if key is None:
            raise ValueError("None may not be used as a key.")
        if key in self._registry:
            raise KeyError("Cannot multiply-register key {}".format(key))
        primaryKey = tuple(getattr(subclass, p, None)
                           for p in self.TEMPLATE_PARAMS)
        if len(primaryKey) == 1:
            # indices are only tuples if there are multiple elements
            primaryKey = primaryKey[0]
        if self._registry.get(primaryKey, None) != subclass:
            raise ValueError("Subclass is not registered with this base class.")
        self._registry[key] = subclass

    # Immutable mapping interface defined below.  We don't use collections
    # mixins because we don't want their comparison operators.

    def __getitem__(self, key):
        return self._registry[key]

    def __iter__(self):
        return iter(self._registry)

    def __len__(self):
        return len(self._registry)

    def __contains__(self, key):
        return key in self._registry

    def keys(self):
        """Return an iterable containing all keys (including aliases).
        """
        return self._registry.keys()

    def values(self):
        """Return an iterable of registered subclasses, with duplicates
        corresponding to any aliases.
        """
        return self._registry.values()

    def items(self):
        """Return an iterable of (key, subclass) pairs.
        """
        return self._registry.items()

    def get(self, key, default=None):
        """Return the subclass associated with the given key (including
        aliases), or ``default`` if the key is not recognized.
        """
        return self._registry.get(key, default)