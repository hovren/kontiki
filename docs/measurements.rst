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

    Measurement of an :class:`.sfm.Observation` instance, using a :class:`.sensors.Camera`.
    `huber_c` specifies the design parameter of the used Huber loss function.

    .. py:attribute:: observation

        The :class:`.sfm.Observation` instance that is measured.

    .. py:attribute:: camera

        The :class:`.sensors.Camera` used.


Camera measurement classes
---------------------------

.. autoclass:: StaticRsCameraMeasurement

.. autoclass:: NewtonRsCameraMeasurement

.. autoclass:: LiftingRsCameraMeasurement

    .. py:attribute:: vt

        The row projection time (to be optimized)


IMU measurements
=================

The IMU measurements share the following base class

.. py:class:: ImuMeasurement(imu, t, x, weight=1)

    A measurement `x` (3D) at time `t` using the specified :class:`.sensors.Imu` instance.
    The scalar `weight` is applied to the residual, before being squared.

    .. py:attribute:: imu

        The :class:`.sensors.Imu` instance used.

    .. py:attribute:: t

        The timestamp of the measurement

    .. py:attribute:: weight

        The (scalar) residual weight

IMU measurement classes
------------------------

.. autoclass:: AccelerometerMeasurement

    .. py:attribute:: a

        Measured acceleration

.. autoclass:: GyroscopeMeasurement

    .. py:attribute:: w

        Measured angular velocity
