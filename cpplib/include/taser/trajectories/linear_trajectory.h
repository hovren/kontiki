//
// Created by hannes on 2017-11-29.
//

#ifndef TASERV2_LINEAR_TRAJECTORY_H
#define TASERV2_LINEAR_TRAJECTORY_H

#include "trajectory.h"

#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <entity/entity.h>
#include <entity/paramstore/dynamic_pstore.h>


namespace taser {
namespace trajectories {

namespace internal {

struct LinearMeta : public entity::MetaData {
  LinearMeta() {};
  LinearMeta(double t0) : t0(t0) {};

  double t0; // Time origin

  size_t NumParameters() const override {
    return 1;
  }
};


template<typename T, typename MetaType>
class LinearView  : public TrajectoryView<T, MetaType> {
  using Vector3 = Eigen::Matrix<T, 3, 1>;
  using Result = std::unique_ptr<TrajectoryEvaluation<T>>;
 public:
  // Inherit constructors
  using TrajectoryView<T, MetaType>::TrajectoryView;

  T t0() const {
    return T(this->meta_.t0);
  }

  void set_t0(double t0) {
    this->meta_.t0 = t0;
  }

  const Vector3 constant() const {
    const Vector3 v = Eigen::Map<const Vector3>(this->pstore_->ParameterData(0));
    return v;
  }

  void set_constant(const Vector3 &k) {
    Eigen::Map<Vector3> kmap(this->pstore_->ParameterData(0));
    kmap = k;
  }

  Result Evaluate(const T t, const int flags) const override {
//    std::cout << "t="<<t<<", t0=" << t0() << ", c=" << constant().transpose() << std::endl;
    auto result = std::make_unique<TrajectoryEvaluation<T>>(flags);
    if (result->needs.Position())
      result->position = constant() * (t - t0());
    if (result->needs.Velocity())
      result->velocity = constant();
    if (result->needs.Acceleration())
      result->acceleration.setZero();
    if (result->needs.Orientation())
      result->orientation = this->calculate_orientation(t);
    if (result->needs.AngularVelocity())
      result->angular_velocity = constant();
    return result;
  }

  double MinTime() const override {
    return -std::numeric_limits<double>::infinity();
  }

  double MaxTime() const override {
    return std::numeric_limits<double>::infinity();
  }

 protected:

  Eigen::Quaternion<T> calculate_orientation(T t) const {
    Vector3 c = constant();
    T theta = c.norm() * (t - t0());
    Vector3 n = c.normalized();
    Eigen::AngleAxis<T> aa(theta, n);
    return Eigen::Quaternion<T>(aa);
  }
}; // ::View

// Entity (in case someone wants to subclass this entity
template<template <typename...> typename ViewTemplate, typename MetaType, typename StoreType>
class LinearEntity : public TrajectoryEntity<ViewTemplate, MetaType, StoreType> {
 public:
  LinearEntity(double t0, const Eigen::Vector3d& k) {
    size_t i = this->pstore_->AddParameter(3); // Add the constant
    this->set_t0(t0);
    this->set_constant(k);
  }

  void AddToProblem(ceres::Problem &problem,
                    time_init_t times,
                    MetaType &meta,
                    std::vector<entity::ParameterInfo<double>> &parameters) const override {
    // Copy meta as it is
    meta = this->meta_;

    // Add constant to list of parameters
    auto param_constant = this->pstore_->Parameter(0);
    problem.AddParameterBlock(param_constant.data, param_constant.size, param_constant.parameterization);

    if (this->IsLocked())
      problem.SetParameterBlockConstant(param_constant.data);

    parameters.push_back(param_constant);
  }
};


} // namespace internal


// Linear trajectory such that we have
// position p(t) = c * (t - t0)
// angular velocity w(t) = c
// Here c is a 3D vector parameter and t0 is the time origin (metadata)
class LinearTrajectory : public internal::LinearEntity<internal::LinearView,
                                                       internal::LinearMeta,
                                                       entity::DynamicParameterStore<double>> {
 public:
  // import constructor
  using internal::LinearEntity<internal::LinearView,
                               internal::LinearMeta,
                               entity::DynamicParameterStore<double>>::LinearEntity;

  static constexpr const char* CLASS_ID = "LinearTrajectory";

};

} // namespace trajectories
} // namespace taser

#endif //TASERV2_LINEAR_TRAJECTORY_H
