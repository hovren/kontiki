//
// Created by hannes on 2017-11-29.
//

#ifndef TASERV2_TRAJECTORY_H
#define TASERV2_TRAJECTORY_H

#include <Eigen/Dense>
#include <memory>
#include <iostream>
#include <vector>

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

template<typename T>
class DataHolderBase {
 public:
  virtual T* Parameter(size_t i) const = 0;
  virtual std::shared_ptr<DataHolderBase<T>> Slice(size_t start, size_t size) const = 0;
};

template<typename T>
class MutableDataHolderBase : public DataHolderBase<T> {
 public:
  // Add parameter and return its index
  virtual size_t AddParameter(size_t ndims) = 0;

  virtual size_t Size() const = 0;
};

template<typename T>
class PointerHolder : public DataHolderBase<T> {
 public:
  PointerHolder(T const* const* data) : data_(data) {};

  T* Parameter(size_t i) const override {
    return (T*) data_[i];
  }

  std::shared_ptr<DataHolderBase<T>> Slice(size_t start, size_t size) const {
//    std::cout << "PointerHolder<T>::Slice(" << start << ", " << size << ")" << std::endl;
    T const* const* ptr = &data_[start];
    auto slice = std::make_shared<PointerHolder<T>>(ptr);
    return slice;
  }

 protected:
  T const* const* data_;
};

template<typename T>
class VectorHolder : public MutableDataHolderBase<T> {
 public:
  ~VectorHolder() {
    for (auto ptr : data_) {
      delete[] ptr;
    }
  }

  T* Parameter(size_t i) const override {
    return data_[i];
  }

  std::shared_ptr<DataHolderBase<T>> Slice(size_t start, size_t size) const {
    throw std::runtime_error("Not implemented for VectorHolder class");
  }

  size_t Size() const override {
    return data_.size();
  }

  size_t AddParameter(size_t ndims) override {
    auto ptr = new T[ndims];
    data_.push_back(ptr);
    std::cout << "Created Parameter ptr=" << ptr << std::endl;
    return data_.size() - 1;
  }

 protected:
  std::vector<T*> data_;
};

// Base class for directories using CRTP
// Used to collect utility functions common to
// all trajectories (Position, ...)
template<typename T, class Derived, class Meta>
class ViewBase {
  using Vector3 = Eigen::Matrix<T, 3, 1>;
  using Quaternion = Eigen::Quaternion<T>;
  using Result = std::unique_ptr<TrajectoryEvaluation<T>>;
 public:

//  ViewBase(T const* const* params, const Meta& meta) : meta_(meta), holder_(new PointerHolder<T>(params)) { };
  ViewBase(std::shared_ptr<DataHolderBase<T>> data_holder, const Meta& meta) : meta_(meta), holder_(data_holder) { };

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

  std::shared_ptr<DataHolderBase<T>> Holder() const {
    return holder_;
  }

 protected:
  std::shared_ptr<DataHolderBase<T>> holder_;
  const Meta& meta_;
};

template<template<typename> typename ViewTemplate>
class TrajectoryBase {
  using Vector3 = Eigen::Vector3d;
  using Quaternion = Eigen::Quaterniond;
  using Result = std::unique_ptr<TrajectoryEvaluation<double>>;
 public:
  template <typename T>
    using View = ViewTemplate<T>;

  using Meta = typename View<double>::Meta;


  TrajectoryBase(MutableDataHolderBase<double>* holder, const Meta& meta) :
      holder_(holder),
      meta_(meta) { };

  TrajectoryBase(MutableDataHolderBase<double>* holder) :
      TrajectoryBase(holder, Meta()) { };



  template <typename T>
  static View<T> Map(T const* const* params, const Meta &meta) {
//    return View<T>(params, meta);
    auto ptr_holder = std::make_shared<PointerHolder<T>>(params);
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

  std::shared_ptr<MutableDataHolderBase<double>> Holder() const {
    return holder_;
  }

  Meta& MetaRef() {
    return meta_;
  }

 protected:
  Meta meta_;
  const std::shared_ptr<MutableDataHolderBase<double>> holder_;
//  const View<double> view_;
};

} // namespace trajectories
} // namespace taser

#endif //TASERV2_TRAJECTORY_H
