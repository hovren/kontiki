#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include <Eigen/Dense>

#include "measurements/position_measurement.h"

namespace py = pybind11;

PYBIND11_MODULE(_position_measurement, m) {
  m.doc() = "Direct position measurement";

  using Class = taser::measurements::PositionMeasurement;
  auto cls = py::class_<Class, std::shared_ptr<Class>>(m, "PositionMeasurement");
  cls.def(py::init<double, const Eigen::Vector3d &>());

  using Class2 = taser::measurements::AnotherMeasurement;
  auto cls2 = py::class_<Class2, std::shared_ptr<Class2>>(m, "AnotherMeasurement");
  cls2.def(py::init<double, const Eigen::Vector3d &>());
}