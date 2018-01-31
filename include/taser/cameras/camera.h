//
// Created by hannes on 2017-03-17.
//

#ifndef TASER_CAMERA_H
#define TASER_CAMERA_H

#include <iostream>

#include <Eigen/Dense>

#include <entity/entity.h>
#include <taser/types.h>

#include "sfm/landmark.h"

namespace taser {
namespace cameras {

template<typename T>
struct RelativePose {
  using Quaternion = Eigen::Quaternion<T>;
  using Vector3 = Eigen::Matrix<T, 3, 1>;

  Quaternion orientation;
  Vector3 translation;
  RelativePose() : orientation(Quaternion::Identity()), translation(Vector3::Zero()) {};
  RelativePose(const Quaternion &q, const Vector3 &p) : orientation(q), translation(p) {};

  template<typename U>
  RelativePose<U> cast() const {
    return RelativePose<U>(orientation.template cast<U>(), translation.template cast<U>());
  }

};

// Base view for all cameras
namespace internal {

struct CameraMeta : public entity::MetaData {
  size_t NumParameters() const override {
    return 0;  // No parameters
  }

  double readout;
  size_t rows;
  size_t cols;
  RelativePose<double> relative_pose;
};

template<typename T, typename MetaType>
class CameraView : public entity::EntityView<T, MetaType> {
  using Vector2 = Eigen::Matrix<T, 2, 1>;
  using Vector3 = Eigen::Matrix<T, 3, 1>;
 public:
  using entity::EntityView<T, MetaType>::EntityView;

  virtual Vector2 Project(const Vector3 &X) const = 0;
  virtual Vector3 Unproject(const Vector2& y) const = 0;

  size_t rows() const {
    return this->meta_.rows;
  }

  void set_rows(size_t r) {
    this->meta_.rows = r;
  }

  size_t cols() const {
    return this->meta_.cols;
  }

  void set_cols(size_t c) {
    this->meta_.cols = c;
  }

  T readout() const {
    return T(this->meta_.readout);
  }

  void set_readout(T r) {
    this->meta_.readout = r;
  }

  const RelativePose<T> relative_pose() const {
    return this->meta_.relative_pose.template cast<T>();
  }

  void set_relative_pose(const RelativePose<T> &pose) {
    this->meta_.relative_pose = pose;
  }

  Vector3 FromTrajectory(const Vector3 &X_trajectory) const {
    auto rel_pose = relative_pose();
    return rel_pose.orientation * X_trajectory + rel_pose.translation;
  };

  Vector3 ToTrajectory(const Vector3 &X_camera) const {
    auto rel_pose = relative_pose();
    return rel_pose.orientation.conjugate() * (X_camera - rel_pose.translation);
  };

};

// Base class for camera entities
template<template<typename...> typename ViewTemplate, typename MetaType, typename StoreType>
class CameraEntity : public type::Entity<ViewTemplate, MetaType, StoreType> {
 public:
  CameraEntity(size_t rows, size_t cols, double readout) {
    this->set_cols(cols);
    this->set_rows(rows);
    this->set_readout(readout);
  }



};

} // namespace internal
} // namespace cameras

namespace type {
template<typename _Entity, typename T>
using Camera = typename entity::type::base::ForView<_Entity, cameras::internal::CameraView, T>;
} // namespace type


} // namespace taser
#endif //TASER_CAMERA_H
