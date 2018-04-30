#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include <Eigen/Dense>

#include "measurement_helper.h"
#include <kontiki/measurements/orientation_measurement.h>

namespace py = pybind11;

PYBIND11_MODULE(_orientation_measurement, m) {
  m.doc() = "Direct orientation measurement";

  using Class = kontiki::measurements::OrientationMeasurement;
  auto cls = py::class_<Class, std::shared_ptr<Class>>(m, "OrientationMeasurement");
  cls.def(py::init<double, const Eigen::Vector4d &>());

  declare_measurement_common<Class>(cls);
}