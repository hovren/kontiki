//
// Created by hannes on 2017-11-29.
//

#ifndef TASERV2_TRAJECTORY_H
#define TASERV2_TRAJECTORY_H

#include <Eigen/Dense>
#include <memory>
#include <iostream>

namespace taser {
namespace trajectories {

struct MetaBase {
  // This meta uses how many parameters?
  virtual int num_parameters() const = 0;
};

template<typename T>
struct TrajectoryEvaluation {
  using Vector3 = Eigen::Matrix<T, 3, 1>;
  Vector3 position;
  Vector3 velocity;
  Vector3 acceleration;
  Eigen::Quaternion<T> orientation;
  Vector3 angular_velocity;
};

enum {
  EvalPosition = 1,
  EvalVelocity = 2,
  EvalAcceleration = 4,
  EvalOrientation = 8,
  EvalAngularVelocity = 16
};

// Base class for directories using CRTP
// Used to collect utility functions common to
// all trajectories (Position, ...)
template<typename T, class Derived>
class ViewBase {
  using Vector3 = Eigen::Matrix<T, 3, 1>;
  using Quaternion = Eigen::Quaternion<T>;
  using Result = std::unique_ptr<TrajectoryEvaluation<T>>;
 public:

  Vector3 Position(T t) const {
    Result result = static_cast<const Derived*>(this)->Evaluate(t, EvalPosition);
    return result->position;
  }

  Vector3 Velocity(T t) const {
    Result result = static_cast<const Derived*>(this)->Evaluate(t, EvalVelocity);
    return result->velocity;
  }

  Vector3 Acceleration(T t) const {
    Result result = static_cast<const Derived*>(this)->Evaluate(t, EvalAcceleration);
    return result->acceleration;
  }

  Quaternion Orientation(T t) const {
    Result result = static_cast<const Derived*>(this)->Evaluate(t, EvalOrientation);
    return result->orientation;
  }

  Vector3 AngularVelocity(T t) const {
    Result result = static_cast<const Derived*>(this)->Evaluate(t, EvalAngularVelocity);
    return result->angular_velocity;
  }

  // Move point from world to trajectory coordinate frame
  Vector3 FromWorld(Vector3 &Xw, T t) {
    Result result = static_cast<const Derived*>(this)->Evaluate(t, EvalPosition | EvalOrientation);
    return result->orientation.conjugate() * (Xw - result->position);
  }

  // Move point from trajectory to world coordinate frame
  Vector3 ToWorld(Vector3 &Xt, T t) {
    Result result = static_cast<const Derived*>(this)->Evaluate(t, EvalPosition | EvalOrientation);
    return result->orientation * Xt + result->position;
  }

};

template<template<typename> typename ViewT>
class TrajectoryBase {
  using Vector3 = Eigen::Vector3d;
  using Quaternion = Eigen::Quaterniond;
  using Result = std::unique_ptr<TrajectoryEvaluation<double>>;
 public:
  using View = ViewT<double>;
  using Meta = typename View::Meta;
  template <typename T>
      using ViewType = ViewT<T>;

  ViewT<double> AsView() const {
    return View(this->data_.data(), meta_);
  }

  Vector3 Position(double t) const {
    return View(this->data_.data(), meta_).Position(t);
  }

  Vector3 Velocity(double t) const {
    return View(this->data_.data(), meta_).Velocity(t);
  }

  Vector3 Acceleration(double t) const {
    return View(this->data_.data(), meta_).Acceleration(t);
  }

  Quaternion Orientation(double t) const {
    return View(this->data_.data(), meta_).Orientation(t);
  }

  Vector3 AngularVelocity(double t) const {
    return View(this->data_.data(), meta_).AngularVelocity(t);
  }

  // Move point from world to trajectory coordinate frame
  Vector3 FromWorld(Vector3 &Xw, double t) {
    return View(this->data_.data(), meta_).FromWorld(Xw, t);
  }

  Vector3 ToWorld(Vector3 &Xw, double t) {
    return View(this->data_.data(), meta_).ToWorld(Xw, t);
  }

  Meta meta_;
  std::vector<double*> data_;
};

} // namespace trajectories
} // namespace taser

#endif //TASERV2_TRAJECTORY_H
