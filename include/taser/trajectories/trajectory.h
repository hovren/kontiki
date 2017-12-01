//
// Created by hannes on 2017-11-29.
//

#ifndef TASERV2_TRAJECTORY_H
#define TASERV2_TRAJECTORY_H

namespace taser {
namespace trajectories {

// Base class for directories using CRTP
// Used to collect utility functions common to
// all trajectories (Position, ...)
template<typename T, class Derived>
class TrajectoryBase {
  using Vector3 = Eigen::Matrix<T, 3, 1>;
 public:
  Vector3 Position(T t) const {
    return static_cast<const Derived*>(this)->position_impl(t);
  }
};


} // namespace trajectories
} // namespace taser

#endif //TASERV2_TRAJECTORY_H
