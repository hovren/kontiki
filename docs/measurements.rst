.. _sec_measurements:

#############
Measurements
#############

.. py:currentmodule:: kontiki.measurements

A measurement defines a residual that is added to the optimization problem.
They share the functionality of the following (unavailable) base class.

.. py:class:: Measurement

    .. py:method:: error(trajectory : Trajectory) -> ndarray

        Compute the error (a ND-vector), before application of any loss function.

    .. py:method:: measure(trajectory : Trajectory) -> ndarray

        The measured quantity, e.g. IMU accelerometer, or camera image projection.

Camera measurements
====================

The camera classes share the following base class:

.. py:class:: CameraMeasurement(camera, observation, huber_c=1)

    Measurement of an :class:`kontiki.sfm.Observation` instance, using a :class:`kontiki.sensors.Camera`.
    `huber_c` specifies the design parameter of the used Huber loss function.

    .. py:attribute:: observation

        The :class:`kontiki.sfm.Observation` instance that is measured.

    .. py:attribute:: camera

        The :class:`kontiki.sensors.Camera` used.


Camera measurement classes
---------------------------

.. autoclass:: StaticRsCameraMeasurement

.. autoclass:: NewtonRsCameraMeasurement

.. autoclass:: LiftingRsCameraMeasurement

    .. py:attribute:: vt

        The row projection time (to be optimized)
