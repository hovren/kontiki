//
// Created by hannes on 2017-04-12.
//

#ifndef TASER_ATAN_H
#define TASER_ATAN_H

#include "pinhole.h"

#include <Eigen/Core>
#include <ceres/ceres.h>

namespace taser {
namespace cameras {

class AtanCamera : public PinholeCamera {
 public:
  static constexpr const char *CLASS_ID = "Atan";
  using CameraMatrix = PinholeCamera::CameraMatrix;

  AtanCamera(int rows, int cols, double readout, const CameraMatrix &K, const Eigen::Vector2d &wc, double gamma)
      : PinholeCamera(rows, cols, readout, K),
        wc_(wc), gamma_(gamma) {};

  AtanCamera(int rows, int cols, double readout)
      : PinholeCamera(rows, cols, readout),
        wc_(0, 0), gamma_(0) {};

  template<typename T>
  Eigen::Matrix<T, 2, 1> Project(const Eigen::Matrix<T, 3, 1> &X) const {
    const T eps = T(1e-32);

    // Common parts
    Eigen::Matrix<T, 2, 1> A = X.head(2)/(X(2) + eps);
    Eigen::Matrix<T, 2, 1> L = A - this->wc_.cast<T>();
    T r = ceres::sqrt(L.squaredNorm() + eps);
    T f = ceres::atan(r*T(gamma_))/T(gamma_);

    Eigen::Matrix<T, 3, 1> Y(T(0), T(0), T(1));
    Y.head(2) = this->wc_.cast<T>() + f*L/r;

    // Apply camera matrix
    // Normalization not needed since Y(2) == 1
    return (this->camera_matrix_.cast<T>()*Y).head(2);
  }

  template<typename T>
  Eigen::Matrix<T, 3, 1> Unproject(const Eigen::Matrix<T, 2, 1> &x) const {
    const T eps = T(1e-32);
    Eigen::Matrix<T, 3, 1> ph(x(0), x(1), T(1.0));
    Eigen::Matrix<T, 3, 1> phn = this->camera_matrix_inverse_.cast<T>()*ph;
    Eigen::Matrix<T, 2, 1> L = phn.head(2) - wc_.cast<T>();

    T r = ceres::sqrt(L.squaredNorm() + eps);
    T f = ceres::tan(r*T(gamma_))/T(gamma_);

    Eigen::Matrix<T, 3, 1> Y(T(0), T(0), T(1));
    Y.head(2) = wc_.cast<T>() + f*L/r;
    return Y;
  }

  Eigen::Vector2d wc() const { return wc_; }
  void set_wc(Eigen::Vector2d &wc) { wc_ = wc; }

  double gamma() const { return gamma_; }
  void set_gamma(double gamma) { gamma_ = gamma; }

 protected:
  Eigen::Vector2d wc_; // Distortion center
  double gamma_; // Distortion parameter

};

} // namespace cameras
} // namespace taser

#endif //TASER_ATAN_H
