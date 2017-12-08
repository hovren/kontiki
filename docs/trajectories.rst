Trajectories
================
.. py:currentmodule:: taser.trajectories

Trajectory base class
--------------------------
All trajectories inherit the methods and attributes from this base class.
Note that the base class is not exported and can thus not be used.

.. py:class:: TrajectoryBase

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



Subclasses
--------------------------
.. autoclass:: taser.trajectories.LinearTrajectory
    :members:
    :exclude-members: position, velocity, acceleration, orientation, angular_velocity, to_world, from_world


.. autoclass:: taser.trajectories.ConstantTrajectory
    :members:
    :exclude-members: position, velocity, acceleration, orientation, angular_velocity, to_world, from_world
