.. _sec_trajectories:

###################
Trajectories
###################
.. py:currentmodule:: kontiki.trajectories

All trajectories inherit the methods and attributes from this base class.
Note that the base class is not exported and can thus not be used.

.. py:class:: Trajectory

    .. py:method:: position(t : float) -> ndarray

        Position (3D vector) at time t

    .. py:method:: velocity(t : float) -> ndarray

        Velocity (3D vector) at time t

    .. py:method:: acceleration(t : float) -> ndarray

        Acceleration (3D vector) at time t

    .. py:method:: orientation(t : float) -> ndarray

        Orientation (unit quaternion) at time t

    .. py:method:: angular_velocity(t : float) -> ndarray

        The angular velocity at time t

    .. py:method:: from_world(X : ndarray) -> ndarray

        Transform a 3D point from the world coordinate frame to the trajectory coordinate frame.

    .. py:method:: to_world(X : ndarray) -> ndarray

        Transform a 3D point from the trajectory to the world coordinate frame.

    .. py:method:: clone() -> Trajectory

        Clone the trajectory

    .. py:attribute:: min_time

        Minimum time (including) that the trajectory is valid

    .. py:attribute:: max_time

        Maximum time (excluding) that the trajectory is valid

    .. py:attribute:: valid_time

        Tuple of (:py:attr:`min_time`, :py:attr:`max_time`)

    .. py:attribute:: locked

        If `True` then the trajectory is unchanged during estimation


Splined trajectories
============================
The splined trajectories are cubic B-splines, defined as in [Qin2000]_.

.. py:class:: SplinedTrajectory(dt=1, t0=0)

        A splined trajectory with knot spacing `dt` and time offset `t0`.
        The time offset `t0` is the first valid time of the spline.

    .. py:attribute:: dt

        Spline knot spacing

    .. py:attribute:: t0

        Spline time offset  (always same as :py:attr:`.Trajectory.min_time`)

    .. py:method:: __len__() -> int

        Number of spline knots and control points

    .. py:method:: __getitem__(k : int) -> ndarray

        Return control point k

    .. py:method:: __setitem__(k : int, cp : ndarray)

        Set control point k

    .. py:method:: append_knot(cp : ndarray)

        Append a new control point (and knot)

    .. py:method:: extend_to(t : float, cp : ndarray)

        Extend the spline to be valid at least up to `t` by appending control points `cp`

Splined trajectory classes
--------------------------

.. autoclass:: kontiki.trajectories.UniformSE3SplineTrajectory
    :members:
    :exclude-members: position, velocity, acceleration, orientation, angular_velocity, to_world, from_world, clone, locked, min_time, max_time, valid_time, append_knot, evaluate, extend_to

.. autoclass:: kontiki.trajectories.UniformR3SplineTrajectory
    :members:
    :exclude-members: position, velocity, acceleration, orientation, angular_velocity, to_world, from_world, clone, locked, min_time, max_time, valid_time, append_knot, evaluate, extend_to

.. autoclass:: kontiki.trajectories.UniformSO3SplineTrajectory
    :members:
    :exclude-members: position, velocity, acceleration, orientation, angular_velocity, to_world, from_world, clone, locked, min_time, max_time, valid_time, append_knot, evaluate, extend_to


Other trajectories
==========================

.. autoclass:: kontiki.trajectories.SplitTrajectory
    :members:
    :exclude-members: position, velocity, acceleration, orientation, angular_velocity, to_world, from_world, clone, locked, min_time, max_time, valid_time

    .. py:method:: SplitTrajectory(r3_dt=1, so3_dt=1, r3_t0=0, so3_t0=0)

        Construct new split trajectory from specified knot spacings and time offsets.

    .. py:method:: SplitTrajectory(r3_trajectory, so3_trajectory)

        Construct split trajectory by wrapping two previously created
        :py:class:`.UniformR3SplineTrajectory` and :py:class:`.UniformSO3SplineTrajectory` instances.


.. rubric:: References

.. [Qin2000]
    Qin, K.
    *General matrix representations for b-splines*
    The Visual Computer, 16(3–4)
