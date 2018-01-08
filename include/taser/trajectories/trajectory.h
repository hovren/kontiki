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

template<typename Derived>
class TrajectoryBase {
  using Vector3 = Eigen::Vector3d;
  using Quaternion = Eigen::Quaterniond;
  using Result = std::unique_ptr<TrajectoryEvaluation<double>>;
  using T = double;
 public:

  Vector3 Position(T t) const {
    using View = typename Derived::template View<double>;
    return View(static_cast<const Derived*>(this)).Position(t);
  }

  Vector3 Velocity(T t) const {
    using View = typename Derived::template View<double>;
    return View(static_cast<const Derived*>(this)).Velocity(t);
  }


  Vector3 Acceleration(T t) const {
    using View = typename Derived::template View<double>;
    return View(static_cast<const Derived*>(this)).Acceleration(t);
  }

  Quaternion Orientation(T t) const {
    using View = typename Derived::template View<double>;
    return View(static_cast<const Derived*>(this)).Orientation(t);
  }

  Vector3 AngularVelocity(T t) const {
    using View = typename Derived::template View<double>;
    return View(static_cast<const Derived*>(this)).AngularVelocity(t);
  }

  // Move point from world to trajectory coordinate frame
  Vector3 FromWorld(Vector3 &Xw, T t) {
    using View = typename Derived::template View<double>;
    return View(static_cast<const Derived*>(this)).FromWorld(Xw, t);
  }

  Vector3 ToWorld(Vector3 &Xw, T t) {
    using View = typename Derived::template View<double>;
    return View(static_cast<const Derived*>(this)).ToWorld(Xw, t);
  }

  std::vector<double*> data_;
};

} // namespace trajectories
} // namespace taser

#endif //TASERV2_TRAJECTORY_H
