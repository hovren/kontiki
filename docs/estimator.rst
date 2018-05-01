.. _sec_estimator:

#####################
Trajectory estimation
#####################

.. py:currentmodule:: kontiki

The estimation of the trajectory, and all auxilliary parameters, are performed through the
:py:class:`TrajectoryEstimator` class.

Behind the scenes, the estimator builds a Ceres-Solver optimization problem that is then solved.

.. py:class:: kontiki.TrajectoryEstimator

    Class that performs trajectory estimation.

    .. py:method:: solve(progress=True, max_iterations=50)

        Solve the estimation problem by minimizing the measurement residuals.
        Returns a :py:class:`.Summary` instance.

        If progress is `True`, then a progress report is written to `stdout` while the optimization in progressing.

        `max_iterations` specify the number of steps the optimizer is allowed to take.

    .. py:method:: add_measurement(m : Measurement)

        Add a measurement to the optimization problem.

        .. warning:: Make sure the measurement m is alive until :py:meth:`solve` has been called,
                     e.g. by putting it in a list.

    .. py:method:: add_callback(func, update_state=False)

        Adds a callback function which is called after every iteration of the optimizer.

        The callback function should accepts a single :py:class:`IterationSummary` instance as argument,
        and return either `None`, or a :py:class:`CallbackReturnType` value.
        `None` is the same as :py:attr:`.CallbackReturnType.Continue`.

        If your callbacks need to access the objects currently being optimized (e.g. the trajectory),
        you must set `update_state=True`.


Estimator helper classes
==========================
The following classes are wrappers around the Ceres-Solver classes of the same name.

.. autoclass:: kontiki.CallbackReturnType
    :members:

.. autoclass:: kontiki.Summary
    :members:
    :undoc-members:

.. autoclass:: kontiki.IterationSummary
    :members:
    :undoc-members:


