.. _sec_sfm:

######################
Structure from motion
######################

.. py:currentmodule:: kontiki.sfm

The structure from motion primitives used by Kontiki are perhaps a bit different from what you are used to.
Instead of modelling landmarks as e.g. a 3D-point in world coordinates, we instead use a combination of
a reference observation and an inverse depth.

This formulation is nice because it is simple to represent landmarks at infinity, and also only require
us to optimize a single scalar (the inverse depth) instead of a full 3D-point.
For a more detailed description see e.g. [Lovegrove2013]_.

Usage
======
To setup a structure from motion problem we create :class:`View` and :class:`Landmark` objects, and then add
:class:`Observation` objects by calling :py:meth:`View.create_observation`.
Note that it is impossible to create :class:`Observation` objects any other way.

Assume that we have a list of tracks that each contain a list of tuples `(n, x)`, where `n` is a frame number,
and `y` is an image point. Then creating the structure from motion objects looks like this::

    landmarks = []
    views = {}  # frame number -> View

    for track_data in tracks:
        lm = Landmark()
        for n, y in track_data:
            view = views.get(n, View(n / frame_rate))
            view.create_observation(lm, y)

To be usable, we also need to set the reference observation for each landmark. Easiest is to use the first one::

    for lm in landmarks:
        lm.reference = lm.observations[0]


Classes
=================

.. autoclass:: View
    :members:

.. autoclass:: Landmark
    :members:

.. autoclass:: Observation
    :members:


References
----------

.. [Lovegrove2013]
    Lovegrove, S.; Patron-Perez, A.; and Sibley, G.
    *Spline Fusion: A continuous-time representation for visual-inertial fusion with application to rolling shutter cameras*
    In Procedings of the British Machine Vision Conference 2013
