########
Overview
########

Kontiki was created to estimate a continuous-time trajectory, and any auxiliary parameters, given a set of measurements.

To use kontiki you need to understand, and create, objects of the following types:

:ref:`sec_trajectories`
    A continuous-time trajectory that you want to estimate

Measurements
    The optimizer tries to estimate the trajectory (and other optimizable entities) such that the measurement errors are minimized.

:ref:`sec_sensors`
    A sensor, such as a camera or an IMU, models how a measurement was created.
    Some sensors have parameters that can be estimated if you want to.

Data objects
    Some measurements are connected to more complex objects, which might in turn be available for estimation.
    A prime example are camera measurements, where the associated landmark is an object that we want to estimate as well.


The trajectory, and set of measurements, are then added to a :py:class:`TrajectoryEstimator` instance that can then minimize
the measurement errors.


Example visual-inertial structure from motion estimation
========================================================
.. code-blocK:: python

    import numpy as np
    import itertools
    from kontiki import trajectories, sensors, TrajectoryEstimator
    import kontiki.measurements as M

    views, landmarks = load_my_sfm_data()

    trajectory = trajectories.UniformSE3SplineTrajectory()
    trajectory.extend_to(some_max_time, np.eye(4))

    imu = sensors.BasicImu()

    camera = sensors.PinholeCamera(720, 1280, 0.03, my_camera_matrix)

    gyro_meas = [M.GyroscopeMeasurement(imu, t, x)
                 for t, x in my_gyro_data]
    acc_meas = [M.AccelerometerMeasurement(imu, t, x)
                for t, x in my_acc_data]
    im_meas = [M.StaticRsCameraMeasurement(camera, obs)
               for v in view for obs in v.observations]

    estimator = TrajectoryEstimator(trajectory)
    for m in itertools.chain(gyro_meas, acc_meas, im_meas):
        estimator.add_measurement(m)

    summary = estimator.solve()
