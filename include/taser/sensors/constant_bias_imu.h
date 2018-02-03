//
// Created by hannes on 2018-01-31.
//

#ifndef TASERV2_CONSTANT_BIAS_IMU_H
#define TASERV2_CONSTANT_BIAS_IMU_H

#include "imu.h"
#include "basic_imu.h"

namespace taser {
namespace sensors {
namespace internal {

struct ConstantBiasImuMeta : public BasicImuMeta {
  size_t NumParameters() const override {
    return 2;
  }
};

template<typename T, typename MetaType>
class ConstantBiasImuView : public ImuView<T, MetaType, ConstantBiasImuView<T, MetaType>> {
  using Vector3 = Eigen::Matrix<T, 3, 1>;
  using Vector3Map = Eigen::Map<Vector3>;
  using Base = ImuView<T, MetaType, ConstantBiasImuView<T, MetaType>>;
 public:
  using Base::ImuView;

  Vector3Map accelerometer_bias() const {
    return Vector3Map(this->holder_->ParameterData(0));
  }

  void set_accelerometer_bias(const Vector3& b) {
    Vector3Map bmap(this->holder_->ParameterData(0));
    bmap = b;
  }

  Vector3Map gyroscope_bias() const {
    return Vector3Map(this->holder_->ParameterData(1));
  }

  void set_gyroscope_bias(const Vector3 &b) {
    Vector3Map bmap(this->holder_->ParameterData(1));
    bmap = b;
  }

  template<typename TrajectoryModel>
  Vector3 Accelerometer(const type::Trajectory<TrajectoryModel, T> &trajectory, T t) const {
    Vector3 base = Base::template StandardAccelerometer<TrajectoryModel>(trajectory, t);
    return base + accelerometer_bias();
  }

  template<typename TrajectoryModel>
  Vector3 Gyroscope(const type::Trajectory<TrajectoryModel, T> &trajectory, T t) const {
    Vector3 base = Base::template StandardGyroscope<TrajectoryModel>(trajectory, t);
    return  base + gyroscope_bias();
  }
};

template<template<typename...> typename ViewTemplate, typename MetaType, typename StoreType>
class ConstantBiasImuEntity : public BasicImuEntity<ViewTemplate, MetaType, StoreType> {
  using Base = BasicImuEntity<ViewTemplate, MetaType, StoreType>;
 public:
  ConstantBiasImuEntity(const Eigen::Vector3d &abias, const Eigen::Vector3d &gbias) :
      gyro_bias_locked_(true),
      acc_bias_locked_(true) {
    // Define parameters
    this->holder_->AddParameter(3); // 0: Accelerometer bias
    this->holder_->AddParameter(3); // 1: Gyroscope bias

    this->set_accelerometer_bias(abias);
    this->set_gyroscope_bias(gbias);
  }

  ConstantBiasImuEntity() :
    ConstantBiasImuEntity(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()) { }

  bool GyroscopeBiasIsLocked() const {
    return gyro_bias_locked_;
  }

  bool LockGyroscopeBias(bool lock) {
    gyro_bias_locked_ = lock;
  }

  bool AccelerometerBiasIsLocked() const {
    return acc_bias_locked_;
  }

  bool LockAccelerometerBias(bool lock) {
    acc_bias_locked_ = lock;
  }


  void AddToProblem(ceres::Problem &problem,
                    time_init_t times,
                    MetaType &meta,
                    std::vector<entity::ParameterInfo<double>> &parameters) const override {
    Base::AddToProblem(problem, times, meta, parameters);

    auto p_ab = this->holder_->Parameter(0);
    problem.AddParameterBlock(p_ab.data, p_ab.size, p_ab.parameterization);
    parameters.push_back(p_ab);

    if (acc_bias_locked_)
      problem.SetParameterBlockConstant(p_ab.data);

    auto p_gb = this->holder_->Parameter(1);
    problem.AddParameterBlock(p_gb.data, p_gb.size, p_gb.parameterization);
    parameters.push_back(p_gb);

    if (gyro_bias_locked_)
      problem.SetParameterBlockConstant(p_gb.data);
  }

 protected:
  bool gyro_bias_locked_;
  bool acc_bias_locked_;
};


} // namespace internal

class ConstantBiasImu : public internal::ConstantBiasImuEntity<internal::ConstantBiasImuView,
                                                               internal::ConstantBiasImuMeta,
                                                               entity::DynamicParameterStore<double>> {
public:
// Inherit constructors
using internal::ConstantBiasImuEntity<internal::ConstantBiasImuView,
                                      internal::ConstantBiasImuMeta,
                                      entity::DynamicParameterStore<double>>::ConstantBiasImuEntity;
static constexpr const char* CLASS_ID = "ConstantBiasImu";
};

} // namespace sensors
} // namespace taser

#endif //TASERV2_CONSTANT_BIAS_IMU_H
