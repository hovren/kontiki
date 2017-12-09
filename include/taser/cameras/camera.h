//
// Created by hannes on 2017-03-17.
//

#ifndef TASER_CAMERA_H
#define TASER_CAMERA_H

#include <iostream>

#include <Eigen/Dense>

#include "sfm/landmark.h"

namespace taser {
namespace cameras {

struct RelativePose {
  Eigen::Quaterniond orientation;
  Eigen::Vector3d translation;
  RelativePose() : orientation(Eigen::Quaterniond::Identity()), translation(Eigen::Vector3d::Zero()) {};
  RelativePose(const Eigen::Quaterniond &q, const Eigen::Vector3d &p) : orientation(q), translation(p) {};
};

template <typename Derived>
class CameraBase {
 public:
  CameraBase(int rows, int cols, double readout)
      : rows_(rows), cols_(cols), readout_(readout) {};

  auto rows() const { return rows_; }
  void set_rows(int r) { rows_ = r; }
  auto cols() const { return cols_; }
  void set_cols(int c) { cols_ = c; }
  double readout() const { return readout_; }
  void set_readout(double r) { readout_ = r; }

  const RelativePose& relative_pose() const {
    return relative_pose_;
  }

  void set_relative_pose(const RelativePose &pose) {
    relative_pose_ = pose;
  }

  template <typename T>
  Eigen::Matrix<T, 3, 1> FromTrajectory(const Eigen::Matrix<T, 3, 1> &X_trajectory) const {
    return relative_pose_.orientation.cast<T>() * X_trajectory + relative_pose_.translation.cast<T>();
  };

  template<typename T>
  Eigen::Matrix<T, 3, 1> ToTrajectory(const Eigen::Matrix<T, 3, 1> &X_camera) const {
    return relative_pose_.orientation.conjugate().cast<T>() * (X_camera - relative_pose_.translation.cast<T>());
  };

 protected:
  int rows_;
  int cols_;
  double readout_;
  RelativePose relative_pose_; // Trajectory->Camera
};

} // namespace cameras
} // namespace taser
#endif //TASER_CAMERA_H
