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

 protected:
  int rows_;
  int cols_;
  double readout_;
};

} // namespace cameras
} // namespace taser
#endif //TASER_CAMERA_H
