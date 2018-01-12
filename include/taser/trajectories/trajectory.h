#ifndef TASERV2_TRAJECTORY_H
#define TASERV2_TRAJECTORY_H

#include <memory>
#include <iostream>
#include <vector>

#include <Eigen/Dense>
#include <ceres/ceres.h>

#include "dataholders/dataholder.h"
#include "dataholders/pointerholder.h"

namespace taser {
namespace trajectories {

// Base class for trajectory metadata
// The metadata should contain everything that is needed to use a trajectory given
// a data holder instance
struct MetaBase {
  // This meta uses how many parameters?
  virtual int NumParameters() const = 0;
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

using time_span_t = std::pair<double, double>;
using time_init_t = std::initializer_list<time_span_t>;

// Base class for trajectory views
// This is used to collect utility functions common to all views (Position, ...)
// Views are intended to be immutable and uses readonly DataHolderBase parameter stores.
template<typename T, class Meta>
class ViewBase {
  static_assert(
      std::is_base_of<MetaBase, Meta>::value,
      "Meta must be subclass of MetaBase"
  );

  using Vector3 = Eigen::Matrix<T, 3, 1>;
  using Quaternion = Eigen::Quaternion<T>;
  using Result = std::unique_ptr<TrajectoryEvaluation<T>>;
 public:

//  ViewBase(T const* const* params, const Meta& meta) : meta_(meta), holder_(new PointerHolder<T>(params)) { };
  ViewBase(std::shared_ptr<dataholders::DataHolderBase<T>> data_holder, const Meta& meta) : meta_(meta), holder_(data_holder) { };

  virtual Result Evaluate(T t, int flags) const = 0;

  Vector3 Position(T t) const {
    return this->Evaluate(t, EvalPosition)->position;
  }

  Vector3 Velocity(T t) const {
    return this->Evaluate(t, EvalVelocity)->velocity;
  }

  Vector3 Acceleration(T t) const {
    return this->Evaluate(t, EvalAcceleration)->acceleration;
  }

  Quaternion Orientation(T t) const {
    return this->Evaluate(t, EvalOrientation)->orientation;
  }

  Vector3 AngularVelocity(T t) const {
    return this->Evaluate(t, EvalAngularVelocity)->angular_velocity;
  }

  // Move point from world to trajectory coordinate frame
  Vector3 FromWorld(Vector3 &Xw, T t) {
    Result result = this->Evaluate(t, EvalPosition | EvalOrientation);
    return result->orientation.conjugate() * (Xw - result->position);
  }

  // Move point from trajectory to world coordinate frame
  Vector3 ToWorld(Vector3 &Xt, T t) {
    Result result = this->Evaluate(t, EvalPosition | EvalOrientation);
    return result->orientation * Xt + result->position;
  }

  std::shared_ptr<dataholders::DataHolderBase<T>> Holder() const {
    return holder_;
  }

 protected:
  std::shared_ptr<dataholders::DataHolderBase<T>> holder_;
  const Meta& meta_;
};


// Base class for trajectories
// A trajectory is the combination of a view, metadata, and a data holder that owns data
template<template<typename> typename ViewTemplate>
class TrajectoryBase {
  using Vector3 = Eigen::Vector3d;
  using Quaternion = Eigen::Quaterniond;
 public:
  template <typename T>
    using View = ViewTemplate<T>;

  using Meta = typename View<double>::Meta;


  TrajectoryBase(dataholders::MutableDataHolderBase<double>* holder, const Meta& meta) :
      holder_(holder),
      meta_(meta) { };

  TrajectoryBase(dataholders::MutableDataHolderBase<double>* holder) :
      TrajectoryBase(holder, Meta()) { };

  virtual void AddToProblem(ceres::Problem& problem,
                    const time_init_t &times,
                    Meta& meta,
                    std::vector<double*> &parameter_blocks,
                    std::vector<size_t> &parameter_sizes) const = 0;


  template <typename T>
  static View<T> Map(T const* const* params, const Meta &meta) {
    auto ptr_holder = std::make_shared<dataholders::PointerHolder<T>>(params);
    return View<T>(ptr_holder, meta);
  }

  View<double> AsView() const {
    return View<double>(holder_, meta_);
  }

  Vector3 Position(double t) const {
    return AsView().Position(t);
  }

  Vector3 Velocity(double t) const {
    return AsView().Velocity(t);
  }

  Vector3 Acceleration(double t) const {
    return AsView().Acceleration(t);
  }

  Quaternion Orientation(double t) const {
    return AsView().Orientation(t);
  }

  Vector3 AngularVelocity(double t) const {
    return AsView().AngularVelocity(t);
  }

  // Move point from world to trajectory coordinate frame
  Vector3 FromWorld(Vector3 &Xw, double t) {
    return AsView().FromWorld(Xw, t);
  }

  Vector3 ToWorld(Vector3 &Xw, double t) {
    return AsView().ToWorld(Xw, t);
  }

  std::shared_ptr<dataholders::MutableDataHolderBase<double>> Holder() const {
    return holder_;
  }

  Meta& MetaRef() {
    return meta_;
  }

 protected:
  Meta meta_;
  const std::shared_ptr<dataholders::MutableDataHolderBase<double>> holder_;
};

} // namespace trajectories
} // namespace taser

#endif //TASERV2_TRAJECTORY_H
