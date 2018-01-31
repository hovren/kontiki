#ifndef TASERV2_IMU_H
#define TASERV2_IMU_H

#include <Eigen/Dense>

#include <entity/entity.h>
#include <taser/trajectories/trajectory.h>
#include <taser/types.h>
#include <taser/constants.h>
#include <entity/paramstore/empty_pstore.h>
#include <entity/paramstore/dynamic_pstore.h>

namespace taser {
namespace sensors {

namespace internal {

static const double STANDARD_GRAVITY = 9.80665;

// Base Imu view using CRTP to access the correct Gyroscope()/Accelerometer() methods
// All IMU views must inherit from this one directly, and not through subclasses.
template<typename T, typename MetaType, typename Derived>
class ImuView : public entity::EntityView<T, MetaType> {
  using Vector3 = Eigen::Matrix<T, 3, 1>;
  using Flags = trajectories::EvaluationFlags;
 public:
  // Import constructor
  using entity::EntityView<T, MetaType>::EntityView;

//  static const Vector3 GRAVITY;

  // Accelerometer measurement (exploits CRTP)
  template<typename TrajectoryModel>
  Vector3 Accelerometer(const type::Trajectory<TrajectoryModel, T> &trajectory, T t) const {
    return static_cast<const Derived*>(this)->template Accelerometer<TrajectoryModel>(trajectory, t);
  }

  // Gyroscope measurement (exploits CRTP)
  template<typename TrajectoryModel>
  Vector3 Gyroscope(const type::Trajectory<TrajectoryModel, T> &trajectory, T t) const {
    return static_cast<const Derived*>(this)->template Gyroscope<TrajectoryModel>(trajectory, t);
  }

 protected:
  // Standard gyroscope function
  template<typename TrajectoryModel>
  Vector3 StandardGyroscope(const type::Trajectory<TrajectoryModel, T> &trajectory, T t) const {
    auto result = trajectory.Evaluate(t, Flags::EvalOrientation | Flags::EvalAngularVelocity);
    // Rotate from world to body coordinate frame
    return result->orientation.conjugate()*result->angular_velocity;
  }

  // Standard gyroscope function
  template<typename TrajectoryModel>
  Vector3 StandardAccelerometer(const type::Trajectory<TrajectoryModel, T> &trajectory, T t) const {
    auto result = trajectory.Evaluate(t, Flags::EvalOrientation | Flags::EvalAcceleration);
    return result->orientation.conjugate() * (result->acceleration + Constants<T>::Gravity);
  }

};

//template<typename T, typename MetaType, typename Derived>
//const Eigen::Matrix<T, 3, 1> ImuView<T, MetaType, Derived>::GRAVITY = Eigen::Matrix<T, 3, 1>(T(0), T(0), T(-STANDARD_GRAVITY));


template<template<typename...> typename ViewTemplate, typename MetaType, typename StoreType>
class ImuEntity : public type::Entity<ViewTemplate, MetaType, StoreType> {
  using Vector3 = Eigen::Vector3d;
 public:
  // Inherit constructors
  using type::Entity<ViewTemplate, MetaType, StoreType>::Entity;
};

} // namespace detail
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
