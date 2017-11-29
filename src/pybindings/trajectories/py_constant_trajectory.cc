//
// Created by hannes on 2017-11-29.
//

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include <Eigen/Dense>

namespace py = pybind11;

PYBIND11_MODULE(_constant_trajectory, m) {
  m.doc() = "Constant Trajectory for testing purposes";

  m.def("testing", []{
    return 6;
  });

} // PYBIND11_MODULE