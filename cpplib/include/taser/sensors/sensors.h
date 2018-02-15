//
// Created by hannes on 2018-02-07.
//

#ifndef TASERV2_SENSORS_H
#define TASERV2_SENSORS_H

#include <Eigen/Dense>

#include <entity/entity.h>
#include <taser/types.h>

namespace taser {
namespace sensors {

namespace internal {

struct SensorMeta : public entity::MetaData {
  size_t NumParameters() const override {
    return 2; // Relative pose
  }
};

template<typename T, typename MetaType>
class SensorView : public entity::EntityView<T, MetaType> {
  using Vector3 = Eigen::Matrix<T, 3, 1>;
  using Vector3Map = Eigen::Map<Vector3>;
  using Quaternion = Eigen::Quaternion<T>;
  using QuaternionMap = Eigen::Map<Quaternion>;
 public:
  // Import constructor
  using entity::EntityView<T, MetaType>::EntityView;

  QuaternionMap relative_orientation() const {
    return QuaternionMap(this->holder_->ParameterData(0));
  }

  void set_relative_orientation(const Quaternion &q) {
    QuaternionMap qmap(this->holder_->ParameterData(0));
    qmap = q;
  }

  Vector3Map relative_position() const {
    return Vector3Map(this->holder_->ParameterData(1));
  }

  void set_relative_position(const Vector3 &p) {
    Vector3Map pmap(this->holder_->ParameterData(1));
    pmap = p;
  }

  Vector3 FromTrajectory(const Vector3 &X_trajectory) const {
    return relative_orientation() * X_trajectory + relative_position();
  };

  Vector3 ToTrajectory(const Vector3 &X_camera) const {
    return relative_orientation().conjugate() * (X_camera - relative_position());
  };
};

template<template<typename...> typename ViewTemplate, typename MetaType, typename StoreType>
class SensorEntity : public type::Entity<ViewTemplate, MetaType, StoreType> {
 public:
  SensorEntity() :
      orientation_parameterization_(new ceres::EigenQuaternionParameterization()),
      relative_position_locked_(true),
      relative_orientation_locked_(true) {
      // 0: Relative orientation
      this->holder_->AddParameter(4, orientation_parameterization_.get());
      // 1: Relative position
      this->holder_->AddParameter(3);

    this->set_relative_position(Eigen::Vector3d::Zero());
    this->set_relative_orientation(Eigen::Quaterniond::Identity());
  };

  void AddToProblem(ceres::Problem &problem,
                    time_init_t times,
                    MetaType &meta,
                    std::vector<entity::ParameterInfo<double>> &parameters) const override {
    auto pi_qct = this->holder_->Parameter(0);
    auto pi_pct = this->holder_->Parameter(1);

    problem.AddParameterBlock(pi_qct.data, pi_qct.size, pi_qct.parameterization);
    parameters.push_back(pi_qct);

    if (relative_orientation_locked_)
      problem.SetParameterBlockConstant(pi_qct.data);

    problem.AddParameterBlock(pi_pct.data, pi_pct.size, pi_pct.parameterization);
    parameters.push_back(pi_pct);

    if (relative_position_locked_)
      problem.SetParameterBlockConstant(pi_pct.data);
  }

 protected:
  bool relative_position_locked_;
  bool relative_orientation_locked_;
  std::unique_ptr<ceres::EigenQuaternionParameterization> orientation_parameterization_;
};


} // namespace internal
} // namespace sensors

// Type specifier to get an Imu instance
namespace type {
template<typename E, typename T>
using Sensor = typename entity::type::base::ForView<E, sensors::internal::SensorView, T>;
}

} // namespace taser


#endif //TASERV2_SENSORS_H
