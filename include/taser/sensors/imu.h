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
  size_t NumParameters() const override {
    return 0;
  }
};

struct ConstantBiasImuMeta : public BasicImuMeta {
  size_t NumParameters() const override {
    return 2;
  }
};

// Base Imu view using CRTP to access the correct Gyroscope()/Accelerometer() methods
// All IMU views must inherit from this one directly, and not through subclasses.
template<typename T, typename MetaType, typename Derived>
class ImuView : public entity::EntityView<T, MetaType> {
  using Vector3 = Eigen::Matrix<T, 3, 1>;
  using Flags = trajectories::EvaluationFlags;
 public:
  // Import constructor
  using entity::EntityView<T, MetaType>::EntityView;

  // Gyroscope measurement (exploits CRTP)
  template<typename TrajectoryModel>
  Vector3 Gyroscope(const type::Trajectory<TrajectoryModel, T> &trajectory, T t) const {
    return static_cast<const Derived*>(this)->template Gyroscope<TrajectoryModel>(trajectory, t);
  }

 protected:
  // Standard gyroscope function
  template<typename TrajectoryModel>
  Vector3 StandardGyroscope(const type::Trajectory<TrajectoryModel, T> &trajectory, T t) const {
    std::cout << "ImuView::StandardGyroscope" << std::endl;
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
class BasicImuView : public ImuView<T, MetaType, BasicImuView<T, MetaType>> {
  using Vector3 = Eigen::Matrix<T, 3, 1>;
  using Base = ImuView<T, MetaType, BasicImuView<T, MetaType>>;
 public:
  using Base::ImuView;

  template<typename TrajectoryModel>
  Vector3 Gyroscope(const type::Trajectory<TrajectoryModel, T> &trajectory, T t) const {
    std::cout << "BasicImu::Gyroscope" << std::endl;
    return Base::template StandardGyroscope<TrajectoryModel>(trajectory, t);
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
} // namespace sensors

// Type specifier to get an Imu instance
namespace type {
template<typename E, typename T>
using Imu = typename sensors::internal::ImuView<T,
                                                typename E::Meta,
                                                typename E::template View<T, typename E::Meta>>;
}

} // namespace taser

#endif //TASERV2_IMU_H
