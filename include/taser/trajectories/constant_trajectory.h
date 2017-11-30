//
// Created by hannes on 2017-11-29.
//

#ifndef TASERV2_CONSTANT_TRAJECTORY_H
#define TASERV2_CONSTANT_TRAJECTORY_H

#include "trajectory.h"

#include <Eigen/Dense>

namespace taser {
namespace trajectories {

template<typename T>
class ConstantTrajectory {
  using Vector3 = Eigen::Matrix<T, 3, 1>;

 public:
  static constexpr const char* CLASS_ID = "Constant";

  ConstantTrajectory(const Vector3& k) : constant_(k) {};

  Vector3 constant() const {
    return constant_;
  }

  void set_constant(const Vector3 &k) {
    constant_ = k;
  }

  Vector3 position(T t) {
    return constant_;
  }

 protected:
  Vector3 constant_;
};

} // namespace trajectories
} // namespace taser

#endif //TASERV2_CONSTANT_TRAJECTORY_H
