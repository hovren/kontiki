#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include <Eigen/Dense>

#include "camera_help.h"
#include "cameras/pinhole.h"

namespace py = pybind11;

namespace C = taser::cameras;

PYBIND11_MODULE(_pinhole_camera, m) {
  using Class = C::PinholeCamera;
  auto cls = py::class_<Class, std::shared_ptr<Class>>(m, "PinholeCamera");

  declare_common<Class>(cls);

  // Declare extra Pinhole related things here
  cls.def(py::init<int, int, double, const Class::CameraMatrix &>());
  cls.def_property("camera_matrix", &Class::camera_matrix, &Class::set_camera_matrix);
}