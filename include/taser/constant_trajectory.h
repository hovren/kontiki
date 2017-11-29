//
// Created by hannes on 2017-11-29.
//

#ifndef TASERV2_CONSTANT_TRAJECTORY_H
#define TASERV2_CONSTANT_TRAJECTORY_H

#include "trajectory.h"

#include <Eigen/Dense>

template<typename T>
class ConstantTrajectory {
  using Vector3 = Eigen::Matrix<T, 3, 1>;

  Vector3 position(T t) {
    return constant_;
  }

 protected:
  Vector3 constant_;
};

#endif //TASERV2_CONSTANT_TRAJECTORY_H
