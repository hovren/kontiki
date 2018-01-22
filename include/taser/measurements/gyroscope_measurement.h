#ifndef TASERV2_GYROSCOPE_MEASUREMENT_H
#define TASERV2_GYROSCOPE_MEASUREMENT_H

#include <Eigen/Dense>

#include <iostream>
#include "trajectory_estimator.h"

namespace taser {
namespace measurements {

class GyroscopeMeasurement {
  using Vector3 = Eigen::Vector3d;
 public:
  GyroscopeMeasurement(double t, const Vector3& w, double weight) :
    t(t), w(w), weight(weight) { };

  GyroscopeMeasurement(double t, const Vector3& w) :
      GyroscopeMeasurement(t, w, 1.0) { };

  template<typename T, typename TrajectoryModel>
  Eigen::Matrix<T, 3, 1> Measure(const typename TrajectoryModel::template View<T> &trajectory) const {
    return trajectory.AngularVelocity(T(t));
  };

  template<typename T, typename TrajectoryModel>
  Eigen::Matrix<T, 3, 1> Error(const typename TrajectoryModel::template View<T> &trajectory) const {
    return w.cast<T>() - Measure<T, TrajectoryModel>(trajectory);
  }

  // Measurement data
  double t;  // Time
  Vector3 w; // Gyroscope angular velocity measurement (rad/s)
  double weight;

 protected:
  template<typename TrajectoryModel>
  struct Residual {
    Residual(const GyroscopeMeasurement &m) : measurement(m) {};

    template <typename T>
    bool operator()(T const* const* params, T* residual) const {
      auto trajectory = TrajectoryModel::template Map<T>(params, meta);
      Eigen::Map<Eigen::Matrix<T,3,1>> r(residual);
      r = measurement.Error<T, TrajectoryModel>(trajectory);
      return true;
    }

    const GyroscopeMeasurement& measurement;
    typename TrajectoryModel::Meta meta;
  }; // Residual;

  template<typename TrajectoryModel>
  void AddToEstimator(taser::TrajectoryEstimator<TrajectoryModel>& estimator) {
    using ResidualImpl = Residual<TrajectoryModel>;
    auto residual = new ResidualImpl(*this);
    auto cost_function = new ceres::DynamicAutoDiffCostFunction<ResidualImpl>(residual);
    std::vector<double*> parameter_blocks;
    std::vector<size_t> parameter_sizes;

    // Add trajectory to problem
    estimator.AddTrajectoryForTimes({{t,t}}, residual->meta, parameter_blocks, parameter_sizes);
    for (auto ndims : parameter_sizes) {
      cost_function->AddParameterBlock(ndims);
    }

    // Add measurement
    cost_function->SetNumResiduals(3);
    estimator.problem().AddResidualBlock(cost_function, nullptr, parameter_blocks);
  }

  // TrajectoryEstimator must be a friend to access protected members
  template<template<typename> typename TrajectoryModel>
  friend class taser::TrajectoryEstimator;
};

} // namespace measurement
} // namespace taser

#endif //TASERV2_GYROSCOPE_MEASUREMENT_H
