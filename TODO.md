## Issues

### Having to lambda-wrap every function taking a base view
We could instead create "fake" view classes for each trajectory, camera, IMU, etc,
and then have the exposed classes point to these as base classes.

### Put relative pose in all sensors
It makes sense in a way. Also make cameras a sensor.
