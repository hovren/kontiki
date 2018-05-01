.. _sec_sensors:

#######
Sensors
#######
.. py:currentmodule:: kontiki.sensors


General
=======

All sensors derive from a common base class.
This base class is not available from Python.

.. py:class:: Sensor

    .. py:attribute:: relative_pose

        A tuple (q, p) where p is the relative position, and q the relative orientation between the sensor coordinate
        frame, and the trajectory coordinate frame such that X_sensor = R * X_world + p.
        q is a unit quaternion representing the orientation R.
        Defaults to the identity orientation, and zero, respectively.

    .. py:attribute:: time_offset

        The time offset `d` between a reference time frame, and the sensor.
        Any function `foo(t)` of the sensor is instead called as `foo(t + d)`.
        Defaults to 0.

    .. py:attribute:: max_time_offset

        When optimizing the time offset, this is the maximum deviation from 0.

    .. py:method:: from_trajectory(X : ndarray) -> ndarray

        Moves the 3D point X from the trajectory to the sensor coordinate frame,
        by applying the relative pose.

    .. py:method:: to_trajectory(X : ndarray) -> ndarray

        Moves the 3D point X from the sensor to the trajectory coordinate frame,
        by applying the relative pose.

    .. py:attribute:: relative_orientation_locked

        If `True`, the orientation part of the relative pose can not be changed during optimization.
        Defaults to True.

    .. py:attribute:: relative_position_locked

        If `True`, the translation part of the relative pose can not be changed during optimization.
        Defaults to True.

    .. py:attribute:: time_offset_locked

        If `True`, the time offset can not be changed during optimization.
        Defaults to True.


Cameras
=======

All cameras share the following (unavailable) base class

.. py:class:: Camera

    .. py:attribute:: rows

        The number of image rows

    .. py:attribute:: cols

        The number of image columns

    .. py:attribute:: readout

        Rolling shutter readout time. Set to 0 for global shutter.

    .. py:method:: project(X : ndarray) -> ndarray

        Projects the 3D point `X` in camera coordinates, to its 2D image plane location using the camera projection model.

        .. note:: This does not apply the relative pose of the camera

    .. py:method:: unproject(y : ndarray) -> ndarray

        Apply inverse projection on image (2D) location y, to get a 3D point X=(x, y, z), where z=1.

        .. note:: This does not apply the relative pose of the camera

    .. py:method:: to_trajectory(X : ndarray) -> ndarray

        Moves the 3D point X in camera coordinates t

Camera classes
-----------------
.. autoclass:: PinholeCamera
    :members: camera_matrix

.. autoclass:: AtanCamera
    :members: wc, gamma

IMUs
============

All inertial measurement units share the following (unavailable) base class:

.. py:class:: IMU

    .. py:method:: accelerometer(trajectory : Trajectory, t : float) -> ndarray

        A single accelerometer measurement at time t, using the specified trajectory.

    .. py:method:: gyroscope(trajectory : Trajectory, t : float) -> ndarray

        A single gyroscope measurement at time t, using the specified trajectory.


IMU classes
-----------------
.. autoclass:: BasicImu

.. autoclass:: ConstantBiasImu
    :members: accelerometer_bias, gyroscope_bias, accelerometer_bias_locked, gyroscope_bias_locked

