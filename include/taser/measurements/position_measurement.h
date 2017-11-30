//
// Created by hannes on 2017-11-29.
//

#ifndef TASERV2_POSITION_MEASUREMENT_H
#define TASERV2_POSITION_MEASUREMENT_H

#include <Eigen/Dense>

#include <iostream>

namespace taser {
namespace measurements {

class PositionMeasurement {
  using Vector3 = Eigen::Vector3d;
 public:
  PositionMeasurement(double t, const Vector3 &p) : t_(t), p_(p) {};

  void hello() {
    std::cout << "Hello from PositionMeasurement!" << std::endl;
  }

 protected:
  double t_;
  Vector3 p_;
};

class AnotherMeasurement {
  using Vector3 = Eigen::Vector3d;
 public:
  AnotherMeasurement(double t, const Vector3 &p) : t_(t), p_(p) {};

  void hello() {
    std::cout << "Hello from AnotherMeasurement!" << std::endl;
  }

 protected:
  double t_;
  Vector3 p_;
};

} // namespace measurements
} // namespace taser


#endif //TASERV2_POSITION_MEASUREMENT_H
