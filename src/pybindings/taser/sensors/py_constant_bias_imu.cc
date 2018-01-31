#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include <Eigen/Dense>

#include <taser/sensors/constant_bias_imu.h>
#include "sensors_helper.h"

namespace py = pybind11;

namespace TS = taser::sensors;

PYBIND11_MODULE(_constant_bias_imu, m) {
  using Class = TS::ConstantBiasImu;
  auto cls = py::class_<Class, std::shared_ptr<Class>>(m, "ConstantBiasImu");

  cls.def(py::init<>());
  cls.def(py::init<Eigen::Vector3d, Eigen::Vector3d>());

  cls.def_property("accelerometer_bias", &Class::accelerometer_bias, &Class::set_accelerometer_bias);
  cls.def_property("gyroscope_bias", &Class::gyroscope_bias, &Class::set_gyroscope_bias);

  declare_imu_common<Class>(cls);
}