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

class PinholeCamera : public CameraBase<PinholeCamera> {
 public:
  static constexpr const char *CLASS_ID = "Pinhole";
  using CameraMatrix = Eigen::Matrix3d;

  PinholeCamera(int rows, int cols, double readout, const CameraMatrix &K)
      : CameraBase(rows, cols, readout),
        camera_matrix_(K), camera_matrix_inverse_(K.inverse()) {};

  PinholeCamera(int rows, int cols, double readout)
      : PinholeCamera{rows, cols, readout, CameraMatrix::Identity()} {};

  template<typename T>
  Eigen::Matrix<T, 2, 1> Project(const Eigen::Matrix<T, 3, 1> &X) const {
    Eigen::Matrix<T, 3, 1> y = camera_matrix_.cast<T>()*X;
    return y.head(2)/y(2);
  }

  template<typename T>
  Eigen::Matrix<T, 3, 1> Unproject(const Eigen::Matrix<T, 2, 1> &x) const {
    Eigen::Matrix<T, 3, 1> xh(x(0), x(1), T(1));
    return camera_matrix_inverse_.cast<T>()*xh;
  }

  CameraMatrix camera_matrix() const { return camera_matrix_; };
  void set_camera_matrix(const CameraMatrix &K) {
    camera_matrix_ = K;
    camera_matrix_inverse_ = K.inverse();
  }

 protected:
  CameraMatrix camera_matrix_;
  CameraMatrix camera_matrix_inverse_;
};

} // namespace cameras
} // namespace taser
#endif //TASER_PINHOLE_CAMERA_H
