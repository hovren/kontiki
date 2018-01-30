#ifndef TASERV2_IMU_H
#define TASERV2_IMU_H

#include <Eigen/Dense>

#include <entity/entity.h>
#include <taser/trajectories/trajectory.h>
#include <taser/types.h>
#include <entity/paramstore/empty_pstore.h>
#include <entity/paramstore/dynamic_pstore.h>

namespace taser {
namespace sensors {

namespace internal {

struct BasicImuMeta : public entity::MetaData {

};

struct ConstantBiasImuMeta : public BasicImuMeta { };

template<typename T, typename MetaType>
class ImuView : public entity::EntityView<T, MetaType> {
  using Vector3 = Eigen::Matrix<T, 3, 1>;
  using Flags = trajectories::EvaluationFlags;
 public:
  // Impot constructor
  using entity::EntityView<T, MetaType>::EntityView;

  // Standard Gyroscope function
  template<typename TrajectoryModel>
  Vector3 DefaultGyroscope(const type::Trajectory<TrajectoryModel, T> &trajectory, T t) const {
    std::cout << "ImuView::Gyroscope" << std::endl;
    auto result = trajectory.Evaluate(t, Flags::EvalOrientation | Flags::EvalAngularVelocity);
    // Rotate from world to body coordinate frame
    return result->orientation.conjugate()*result->angular_velocity;
  }
};

template<template<typename...> typename ViewTemplate, typename MetaType, typename StoreType>
class ImuEntity : public type::Entity<ViewTemplate, MetaType, StoreType> {
  using Vector3 = Eigen::Vector3d;
 public:
  // Inherit constructors
  using type::Entity<ViewTemplate, MetaType, StoreType>::Entity;
};

template<typename T, typename MetaType>
class BasicImuView : public ImuView<T, MetaType> {
  using Vector3 = Eigen::Matrix<T, 3, 1>;
  using Base = ImuView<T, MetaType>;
 public:
  using Base::ImuView;

  template<typename TrajectoryModel>
  Vector3 Gyroscope(const type::Trajectory<TrajectoryModel, T> &trajectory, T t) const {
    return this->template DefaultGyroscope<TrajectoryModel>(trajectory, t);
  }
};

template<template<typename...> typename ViewTemplate, typename MetaType, typename StoreType>
class BasicImuEntity : public ImuEntity<ViewTemplate, MetaType, StoreType> {
 public:
  using ImuEntity<ViewTemplate, MetaType, StoreType>::ImuEntity;

 private:
  void AddToProblem(ceres::Problem &problem,
                    time_init_t times,
                    MetaType &meta,
                    std::vector<entity::ParameterInfo<double>> &parameters) const override {
    // No parameters to add
  }
};



template<typename T, typename MetaType>
class ConstantBiasImuView : public BasicImuView<T, MetaType> {
  using Vector3 = Eigen::Matrix<T, 3, 1>;
  using Vector3Map = Eigen::Map<Vector3>;
  using Base = BasicImuView<T, MetaType>;
 public:
  using Base::BasicImuView;

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
    Vector3 base = Base::template DefaultGyroscope<TrajectoryModel>(trajectory, t);
    return  base + gyro_bias();
  }
};

template<template<typename...> typename ViewTemplate, typename MetaType, typename StoreType>
class ConstantBiasImuEntity : public BasicImuEntity<ViewTemplate, MetaType, StoreType> {
 public:
  ConstantBiasImuEntity(const Eigen::Vector3d &gbias, const Eigen::Vector3d &abias) {
    // Define parameters
    this->holder_->AddParameter(3); // 0: Accelerometer bias
    this->holder_->AddParameter(3); // 1: Gyroscope bias

    this->set_gyro_bias(gbias);
  }
};

} // namespace detail

class BasicImu : public internal::BasicImuEntity<internal::BasicImuView,
                                                 internal::BasicImuMeta,
                                                 entity::EmptyParameterStore<double>> {
 public:
  static constexpr const char* CLASS_ID = "BasicImu";
};

class ConstantBiasImu : public internal::ConstantBiasImuEntity<internal::ConstantBiasImuView,
                                                               internal::ConstantBiasImuMeta,
                                                               entity::DynamicParameterStore<double>> {
 public:
  // Inherit constructors
  using internal::ConstantBiasImuEntity<internal::ConstantBiasImuView,
                                        internal::ConstantBiasImuMeta,
                                        entity::DynamicParameterStore<double>>::ConstantBiasImuEntity;
};

//class ConstantBiasImu : public detail::ImuBase<detail::ConstantBiasImuView> {
//  using Vector3 = Eigen::Vector3d;
//  using Vector3Map = Eigen::Map<Vector3>;
// public:
//  static constexpr const char* CLASS_ID = "ConstantBiasImu";
//
//  ConstantBiasImu() :
//    detail::ImuBase<detail::ConstantBiasImuView>(new trajectories::dataholders::VectorHolder<double>()) {
//    this->holder_->AddParameter(3); // 0: Accelerometer bias
//    this->holder_->AddParameter(3); // 1: Gyroscope bias
//  };
//
//  Vector3 GyroscopeBias() const {
//    return AsView().GyroscopeBias();
//  }
//
//  void set_GyroscopeBias(const Vector3 &b) {
//    AsView().GyroscopeBias() = b;
//  }
//
//  // Add trajectory to problem for a set of time spans
//  // Implementers can assume that the the list of time spans is ordered
//  void AddToProblem(ceres::Problem& problem,
//                            const time_init_t &times,
//                            Meta& meta,
//                            std::vector<double*> &parameter_blocks,
//                            std::vector<size_t> &parameter_sizes) const override {
//
//    for (auto i : {0, 1}) {
//      auto ptr = this->holder_->Parameter(i);
//      int size = 3;
//      problem.AddParameterBlock(ptr, size);
//      parameter_blocks.push_back(ptr);
//      parameter_sizes.push_back(size);
//    }
//  }
//
//};

} // namespace sensors

namespace type {
template<typename _Entity, typename T>
using Imu = typename entity::type::base::ForView<_Entity, sensors::internal::ImuView, T>;
}

} // namespace taser

#endif //TASERV2_IMU_H
