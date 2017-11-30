//
// Created by hannes on 2017-11-29.
//

#ifndef TASERV2_LINEAR_TRAJECTORY_H
#define TASERV2_LINEAR_TRAJECTORY_H

#include "trajectory.h"

#include <Eigen/Dense>

namespace taser {
namespace trajectories {

template<typename T>
class LinearTrajectory {
  using Vector3 = Eigen::Matrix<T, 3, 1>;

 public:
  static constexpr const char* CLASS_ID = "Linear";

  LinearTrajectory(const Vector3& k) : constant_(k) {};

  Vector3 constant() const {
    return constant_;
  }

  void set_constant(const Vector3 &k) {
    constant_ = k;
  }

  Vector3 position(T t) {
    return constant_ * t;
  }

 protected:
  Vector3 constant_;
};

} // namespace trajectories
} // namespace taser

#endif //TASERV2_LINEAR_TRAJECTORY_H
