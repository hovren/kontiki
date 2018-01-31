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
