//
// Created by hannes on 2017-03-20.
//

#ifndef TASER_PINHOLE_CAMERA_H
#define TASER_PINHOLE_CAMERA_H

#include <iostream>

#include <Eigen/Dense>

#include "camera.h"

namespace taser {
namespace cameras {

namespace internal {

struct PinholeMeta : public CameraMeta {
  size_t NumParameters() const override {
    return CameraMeta::NumParameters();
  }

  Eigen::Matrix3d camera_matrix; // FIXME: This should be a set of parameters
};

template<typename T, typename MetaType>
class PinholeView : public CameraView<T, MetaType> {
  using CameraMatrix = Eigen::Matrix<T, 3, 3>;
  using Vector3 = Eigen::Matrix<T, 3, 1>;
  using Vector2 = Eigen::Matrix<T, 2, 1>;
 public:
  // Inherit constructors
  using CameraView<T, MetaType>::CameraView;

  CameraMatrix camera_matrix() const {
    return this->meta_.camera_matrix.template cast<T>();
  }

  void set_camera_matrix(const CameraMatrix& K) {
    this->meta_.camera_matrix = K.template cast<double>();
  }

  Vector2 Project(const Vector3 &X) const override {
    Vector3 y = camera_matrix() * X;
    return y.head(2)/y(2);
  }
  Vector3 Unproject(const Vector2 &y) const override {
    Vector3 xh(y(0), y(1), T(1));
    CameraMatrix K = camera_matrix();
    return camera_matrix().inverse() * xh;
  }
};

template<template<typename...> typename ViewTemplate, typename MetaType, typename StoreType>
class PinholeEntity : public CameraEntity<ViewTemplate, MetaType, StoreType> {
  using Base = CameraEntity<ViewTemplate, MetaType, StoreType>;
 public:
  // Inherit constructors
  PinholeEntity(size_t cols, size_t rows, double readout) :
      Base(cols, rows, readout) {
    this->set_camera_matrix(Eigen::Matrix3d::Identity());
  }

  void AddToProblem(ceres::Problem &problem,
                    time_init_t times,
                    MetaType &meta,
                    std::vector<entity::ParameterInfo<double>> &parameters) const override {
    meta = this->meta_;
  }
};

} // namespace internal



class PinholeCamera : public internal::PinholeEntity<internal::PinholeView,
                                                     internal::PinholeMeta,
                                                     entity::DynamicParameterStore<double>> {
 public:
  using internal::PinholeEntity< internal::PinholeView,
                                 internal::PinholeMeta,
                                 entity::DynamicParameterStore<double>>::PinholeEntity;

  static constexpr const char *ENTITY_ID = "Pinhole";
};

} // namespace cameras
} // namespace taser
#endif //TASER_PINHOLE_CAMERA_H
