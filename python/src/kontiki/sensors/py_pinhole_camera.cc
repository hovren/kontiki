#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include <Eigen/Dense>

#include "camera_help.h"
#include "kontiki/sensors/pinhole_camera.h"

namespace py = pybind11;

namespace C = kontiki::sensors;

PYBIND11_MODULE(_pinhole_camera, m) {
  using Class = C::PinholeCamera;
  auto cls = py::class_<Class, std::shared_ptr<Class>>(m, "PinholeCamera");
  cls.doc() = R"pbdoc(Standard pinhole camera
  )pbdoc";

  declare_camera_common<Class>(cls);

  // Declare extra Pinhole related things here
  cls.def(py::init<size_t, size_t, double, const Class::CameraMatrix &>());
  cls.def_property("camera_matrix", &Class::camera_matrix, &Class::set_camera_matrix, "The 3x3 internal camera calibration matrix");
}