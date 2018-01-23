#ifndef TASERV2_GYROSCOPE_MEASUREMENT_H
#define TASERV2_GYROSCOPE_MEASUREMENT_H

#include <Eigen/Dense>

#include <iostream>

#include <taser/trajectories/trajectory.h>
//#include <taser/sensors/imu.h>
#include "trajectory_estimator.h"

namespace taser {
namespace measurements {

using trajectories::TrajectoryView;
using trajectories::TrajectoryMap;
using sensors::ImuView;
using sensors::ImuMap;

template<typename ImuModel>
class GyroscopeMeasurement {
  using Vector3 = Eigen::Vector3d;
 public:
  GyroscopeMeasurement(std::shared_ptr<ImuModel> imu, double t, const Vector3& w, double weight) :
    imu_(imu), t(t), w(w), weight(weight) { };

  GyroscopeMeasurement(std::shared_ptr<ImuModel> imu, double t, const Vector3& w) :
      GyroscopeMeasurement(imu, t, w, 1.0) { };

  template<typename TrajectoryModel, typename T>
  Eigen::Matrix<T, 3, 1> Measure(const ImuView<ImuModel, TrajectoryModel, T> &imu, const TrajectoryView<TrajectoryModel, T> &trajectory) const {
//    using Flags = taser::trajectories::EvaluationFlags;
//    auto result = trajectory.Evaluate(T(t), Flags::EvalOrientation | Flags::EvalAngularVelocity);
//    // Rotate angular velocity to body coordinate frame
//    return result->orientation * result->angular_velocity;
    return imu.Gyroscope(trajectory, T(t));
  };

  template<typename TrajectoryModel, typename T>
  Eigen::Matrix<T, 3, 1> Error(const ImuView<ImuModel, TrajectoryModel, T> &imu, const TrajectoryView<TrajectoryModel, T> &trajectory) const {
    return w.cast<T>() - Measure<TrajectoryModel, T>(imu, trajectory);
  }

  // Data
  std::shared_ptr<ImuModel> imu_;

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
      size_t offset = 0;
      const auto trajectory = TrajectoryMap<TrajectoryModel, T>(&params[offset], trajectory_meta);
      offset += trajectory_meta.NumParameters();
      const auto imu = ImuMap<ImuModel, TrajectoryModel, T>(&params[offset], imu_meta);

      Eigen::Map<Eigen::Matrix<T,3,1>> r(residual);
      r = measurement.Error<T, TrajectoryModel>(trajectory, imu);
      return true;
    }

    const GyroscopeMeasurement& measurement;
    typename ImuModel::Meta imu_meta;
    typename TrajectoryModel::Meta trajectory_meta;
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
