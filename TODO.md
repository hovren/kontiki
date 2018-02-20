## To fix
- Relative pose for cameras are not applied in project/unproject.
Currently they can't, because of inverse depth, but we could make it use
homogeneous coordinates instead.

- IMU does not apply relative pose at all.

## Issues

### Having to lambda-wrap every function taking a base view
We could instead create "fake" view classes for each trajectory, camera, IMU, etc,
and then have the exposed classes point to these as base classes.