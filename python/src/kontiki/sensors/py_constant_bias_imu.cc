#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include <Eigen/Dense>

#include <kontiki/sensors/constant_bias_imu.h>
#include "imu_helper.h"

namespace py = pybind11;

namespace TS = kontiki::sensors;

PYBIND11_MODULE(_constant_bias_imu, m) {
  using Class = TS::ConstantBiasImu;
  auto cls = py::class_<Class, std::shared_ptr<Class>>(m, "ConstantBiasImu");
  cls.doc() = R"pbdoc( IMU with constant biases

  Both biases are modeled as constants.
  )pbdoc";

  cls.def(py::init<>());
  cls.def(py::init<Eigen::Vector3d, Eigen::Vector3d>());

  cls.def_property("accelerometer_bias", &Class::accelerometer_bias, &Class::set_accelerometer_bias,
    "Accelerometer bias");
  cls.def_property("gyroscope_bias", &Class::gyroscope_bias, &Class::set_gyroscope_bias,
    "Gyroscope bias");
  cls.def_property("accelerometer_bias_locked", &Class::AccelerometerBiasIsLocked, &Class::LockAccelerometerBias,
    "If True (default), then the accelerometer bias is not changed during optimization.");
  cls.def_property("gyroscope_bias_locked", &Class::GyroscopeBiasIsLocked, &Class::LockGyroscopeBias,
    "If True (default), then the gyrocope bias is not changed during optimization.");

  declare_imu_common<Class>(cls);
}