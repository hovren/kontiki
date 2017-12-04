#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include <Eigen/Dense>

#include "camera_help.h"
#include "cameras/atan.h"

namespace py = pybind11;

namespace C = taser::cameras;

static void declare_atan(py::module &m) {
  using Class = C::AtanCamera;
  using BaseClass = C::PinholeCamera;
  auto cls = py::class_<Class, BaseClass, std::shared_ptr<Class>>(m, "AtanCamera");

  // All templated common functions must be explicitly created since they are not inherited
  // from the Pinhole base class.
  declare_common<Class>(cls);

  cls.def(py::init<int, int, double, const Class::CameraMatrix&, const Eigen::Vector2d&, double >());
  cls.def_property("wc", &Class::wc, &Class::set_wc, "Distortion center");
  cls.def_property("gamma", &Class::gamma, &Class::set_gamma, "Distortion parameter");
}

PYBIND11_MODULE(_atan_camera, m) {
  py::module::import("taser.cameras._pinhole_camera");
  declare_atan(m);
}