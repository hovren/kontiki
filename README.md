Kontiki - the continuous time toolkit
====================================

Kontiki is a toolkit for continuous-time structure from motion.
In short, it can estimate a trajectory (and 3D structure) from a set of measurements.

The documentation is available at https://hovren.github.io/kontiki/.

Example: Visual-inertial structure from motion
---------------------------------------------

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
    
Installation
============
See the documentation.

