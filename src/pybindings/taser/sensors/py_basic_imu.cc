#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include <Eigen/Dense>

#include <taser/sensors/imu.h>

namespace py = pybind11;

namespace TS = taser::sensors;

PYBIND11_MODULE(_basic_imu, m) {
  using Class = TS::BasicImu;
  auto cls = py::class_<Class, std::shared_ptr<Class>>(m, "BasicImu");

  cls.def(py::init<>());
}