//
// Created by hannes on 2017-03-17.
//

#ifndef TASER_CAMERA_H
#define TASER_CAMERA_H

#include <iostream>

#include <Eigen/Dense>

#include <entity/entity.h>
#include "sensors.h"
#include <taser/types.h>

#include "sfm/landmark.h"

namespace taser {
namespace sensors {

// Base view for all cameras
namespace internal {

struct CameraMeta : public SensorMeta {
  double readout;
  size_t rows;
  size_t cols;
};

template<typename T>
struct CameraEvaluation {
  using Vector2 = Eigen::Matrix<T, 2, 1>;

  CameraEvaluation(bool derive) :
      derive(derive) { };

  // Data
  Vector2 y;
  Vector2 dy;

  bool derive;
};

template<typename T, typename MetaType>
class CameraView : public SensorView<T, MetaType> {
 protected:
  using Vector2 = Eigen::Matrix<T, 2, 1>;
  using Vector3 = Eigen::Matrix<T, 3, 1>;
  using Result = std::unique_ptr<CameraEvaluation<T>>;
 public:
  using SensorView<T, MetaType>::SensorView;

  virtual Result EvaluateProjection(const Vector3 &X, const Vector3 &dX, bool derive) const = 0;

  Vector2 Project(const Vector3 &X) const {
    Vector3 dX;
    auto result = EvaluateProjection(X, dX, false);
    return result->y;
  };

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
};

// Base class for camera entities
template<template<typename...> typename ViewTemplate, typename MetaType, typename StoreType>
class CameraEntity : public SensorEntity<ViewTemplate, MetaType, StoreType> {
 public:
  CameraEntity(size_t rows, size_t cols, double readout) {
    this->set_cols(cols);
    this->set_rows(rows);
    this->set_readout(readout);
  }



};

} // namespace internal
} // namespace sensors

namespace type {
template<typename _Entity, typename T>
using Camera = typename entity::type::base::ForView<_Entity, sensors::internal::CameraView, T>;
} // namespace type


} // namespace taser
#endif //TASER_CAMERA_H
