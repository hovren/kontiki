#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include <Eigen/Dense>

#include "measurement_helper.h"
#include <kontiki/measurements/position_measurement.h>

namespace py = pybind11;

PYBIND11_MODULE(_position_measurement, m) {
  m.doc() = "Direct position measurement";

  using Class = kontiki::measurements::PositionMeasurement;
  auto cls = py::class_<Class, std::shared_ptr<Class>>(m, "PositionMeasurement");
  cls.def(py::init<double, const Eigen::Vector3d &>());

  declare_measurement_common<Class>(cls);
}