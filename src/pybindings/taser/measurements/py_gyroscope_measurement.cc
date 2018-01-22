#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include <Eigen/Dense>

#include "measurement_helper.h"
#include "measurements/gyroscope_measurement.h"

namespace py = pybind11;

PYBIND11_MODULE(_gyroscope_measurement, m) {
  m.doc() = "Gyroscope measurement";

  using Class = taser::measurements::GyroscopeMeasurement;
  auto cls = py::class_<Class, std::shared_ptr<Class>>(m, "GyroscopeMeasurement");
  cls.def(py::init<double, const Eigen::Vector3d &, double>());
  cls.def(py::init<double, const Eigen::Vector3d &>());

  declare_measurement_common<Class>(cls);
}