#################
Extending Kontiki
#################

Kontiki can look tricky to work with because of its close ties to C++ and Ceres-Solver, which makes some parts of the
code base a bit complicated.
Here we will try to explain the steps necessary to implement new functionality.
This documentation is not a tutorial, but gives a high-level view of the steps involve to create new
trajectories, measurements, etc.
For details, look at the source code for an existing class of the type you wish to implement.

.. contents::

General
====================

Extending by forking
--------------------
The first thing to note is that to extend Kontiki you must currently fork it, and implement your new features.
This is because making an extension system that allows external modules is very tricky.
Ideas on how to solve it are very welcome!

Views and Entitites
-------------------
To make it easier to work with Ceres-Solver, our optimization backend, we have implemented a general system for
creating optimizable entities.
To do so we introduce two concepts:

Views
    The View is the workhorse of the entity system.
    It implements all the behaviour of an object, as well as data accessors.
    The view contents are defined by a :cpp:class:`ParameterStore`, and a :cpp:class:`Metadata` instance.

Parameter stores
    The parameter store is an abstraction layer to allow the view to access the actual data required, from different sources.

Metadata
    The metadata holds all information necessary to create/define the object together with the data in the parmaeter store.
    The metadata instance is used when adding a residual to the optimization problem, in order to allow it to be recreated
    at evaluation time.

Entities
    Where the View is created on the fly by wrapping a :cpp:class:`ParameterStore`, and a :cpp:class:`Metadata`,
    the Entity is a concrete instantiation of it.
    It is the interface which users are supposed to instantiate and work with.
    Entities own their data, but Views do not.

As an example, the :py:class:`kontiki.trajectories.UniformSE3SplineTrajectory` is an entity.
The entity instance contains an owning parameter store, and a metadata object that defines the full trajectory.
When we add a measurement that uses the splined trajectory, the residual struct is given a metadata object that defines
the *spline segment* (number of knots, start time, knot spacing) which is required by the measurement.
When the optimizer calls the residual cost function, a View of this spline segment is created from the metadata object,
and a parameter vector.

.. _sec_new_trajectories:

New trajectories
================

To create a new directory the following steps should be performed.

#. Create a new ``my_trajectory.h`` file in ``cpplib/include/kontiki/trajectories/``
#. Define what kind of metadata your new trajectory requires. In short this means to define
    1. Any data that is not a parameter to optimize, e.g. spline knot spacing.
    2. Information required to recreate (part of) the trajectory, e.g. the number of spline segment control points.
    3. Implement the ``NumParameters()`` function that computes the number of parameters used by this metadata object.
#. Create your view template ``MyTrajectoryView<T, MetaType>``, that inherits from ``TrajectoryView<T, MetaType>``.
   Implement getter and setter functions to access data in either the view metadata, or parameter store.
#. Implement ``MyTrajectoryView::Evaluate(const T t, const int flags)``.
   Begin by creating the ``TrajectoryEvaluation`` to be returned, by giving it the ``flags`` argument.
   Fill ``result.position``, ``result.velocity``, etc. with whatever result your trajectory should give at time t.
#. Implement  ``MyTrajectory::MinTime()`` and ``MyTrajectory::MaxTime()``.
#. Implement ``MyTrajectoryEntity<...>`` by inheriting from ``TrajectoryEntity<...>``.
   Here you must define
   1. The constructor(s), used by users of your code to construct the object.
   2. The ``AddToProblem(...)`` method, which adds (part of) the trajectory to the optimization problem.
#. Create ``MyTrajectory`` class by inheriting from ``LinearEntity``.
   Here you must...
   1. add the constructor by ``using MyTrajectoryEntity<...>::MyTrajectoryEntity``.
   2. Define the trajectory ID, ``static constexpr const char* CLASS_ID = "MyTrajectory"``

For Python bindings you also need to

#. Create a new ``py_my_trajectory.cc`` file in ``python/src/kontiki/trajectories/``
#. Wrap the class using pybind11.
#. Add ``MyTrajectory`` to the trajectory list in ``python/src/kontiki/trajectory_defs.h``.
   This makes sure support for the trajectory is compiled into all sensors, estimators, etc.
#. Add ``kontiki_add_module(kontiki.trajectories._my_trajectory)`` to ``pyhon/CMakeLists.txt``.
#. Add ``TrajectoryExtension('_my_measurement')`` to the extensions list in ``pyhon/setup.py``.

New measurements
================

#. Create a new ``my_measurement.h`` file in ``cpplib/include/kontiki/measurements/``
#. Implement your measurement class ``MyMeasurement``.
   The class may need to be templated, e.g. as ``MyMeasurement<ImuModel>`` if it needs an IMU.
#. Implement the templated measurement function ``MyMeasurement::Measure<TrajectoryModel, T>()`` with
   trajectory as the only argument.
#. Similarly implement the templated error function ``MyMeasurement::Error<TrajectoryModel, T>()`` which calls
   the measurement function.
#. Implement the residual struct ``MyMeasurement::Residual<TrajectoryModel>``.
   As per the Ceres-Solver documentation, it must have a method ``Residual::operator<T>()(...)``
   which evaluates the residual. This function should unpack the current trajectory, and any other entity
   used by the residual, and compute the residual value (using ``::Error()`` defined above).
#. Implement ``MyMeasurement::AddToEstimator<TrajectoryModel>()``.
   This function should create the ``MyMeasurement::Residual`` instance, and give it to
   the ``ceres::Problem`` contained in the :py:class:`kontiki.TrajectoryEstimator`.
#. Declare ``kontiki::TrajectoryEstimator`` a ``friend class``.

For Python bindings you also need to

#. Create a new ``py_my_measurement.cc`` file in ``python/src/kontiki/measurements/``
#. Wrap the class using pybind11.
#. Add ``MyMeasurement`` to the correct measurement list in ``python/src/kontiki/measurement_defs.h``.
   This makes sure support for the measurement is compiled into the trajectory estimator.
#. Add ``kontiki_add_module(kontiki.measurements._my_measurement)`` to ``pyhon/CMakeLists.txt``.
#. Add ``MeasurementExtension('_my_measurement')`` to the extensions list in ``pyhon/setup.py``.

New sensors
===========
Implementing new sensors are done mostly the same as when implementing :ref:`sec_new_trajectories`.
If you are implementing a sensor which is a specialization of *Foo*, then you should implement
a new View and Entity that inherits `FooView` and `FooEntity`, respectively.
For the Python bindings, you need to add your newly created class to the correct `xxx_defs.h` file
to make sure support for it is compiled into the system.
