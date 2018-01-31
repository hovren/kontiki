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

  Vector3Map gyro_bias() const {
    return Vector3Map(this->holder_->ParameterData(1));
  }

  void set_gyro_bias(const Vector3& b) {
    Vector3Map bmap(this->holder_->ParameterData(1));
    bmap = b;
  }

  template<typename TrajectoryModel>
  Vector3 Gyroscope(const type::Trajectory<TrajectoryModel, T> &trajectory, T t) const {
    std::cout << "ConstantBiasImuView::Gyroscope" << std::endl;
    Vector3 base = Base::template StandardGyroscope<TrajectoryModel>(trajectory, t);
    return  base + gyro_bias();
  }
};

template<template<typename...> typename ViewTemplate, typename MetaType, typename StoreType>
class ConstantBiasImuEntity : public BasicImuEntity<ViewTemplate, MetaType, StoreType> {
  using Base = BasicImuEntity<ViewTemplate, MetaType, StoreType>;
 public:
  ConstantBiasImuEntity(const Eigen::Vector3d &gbias, const Eigen::Vector3d &abias) {
    // Define parameters
    this->holder_->AddParameter(3); // 0: Accelerometer bias
    this->holder_->AddParameter(3); // 1: Gyroscope bias

    this->set_gyro_bias(gbias);
  }

  void AddToProblem(ceres::Problem &problem,
                    time_init_t times,
                    MetaType &meta,
                    std::vector<entity::ParameterInfo<double>> &parameters) const override {
    Base::AddToProblem(problem, times, meta, parameters);
    for (auto i : {0 , 1}) {
      auto pi = this->holder_->Parameter(i);
      problem.AddParameterBlock(pi.data, pi.size, pi.parameterization);
      parameters.push_back(pi);
    }
  }
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
