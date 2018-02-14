//
// Created by hannes on 2017-12-04.
//

#ifndef TASERV2_CAMERA_HELP_H
#define TASERV2_CAMERA_HELP_H

#include "taser/sensors/camera.h"

#include <Eigen/Dense>

#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "sensors_helper.h"

namespace py = pybind11;

namespace C = taser::sensors;

using Vector3 = Eigen::Vector3d;
using Vector2 = Eigen::Vector2d;

template<typename Class, typename PyClass>
void declare_camera_common(PyClass &cls) {

  declare_sensors_common<Class>(cls);

  cls.def(py::init<size_t, size_t, double>());
  cls.def("project", &Class::Project, "Project");
  cls.def("unproject", &Class::Unproject, "Unproject");
  cls.def("evaluate_projection", [](Class &self, const Vector3 &X, const Vector3 &dX, bool derive){
    auto result = self.EvaluateProjection(X, dX, derive);
    auto pair = std::pair<Vector2, Vector2>(result->y, result->dy);
    return pair;
  }, "Evaluate");
  cls.def_property("readout",
                   &Class::readout,
                   &Class::set_readout,
                   "Rolling shutter readout time");
  cls.def_property("rows",
                   &Class::rows,
                   &Class::set_rows,
                   "Image rows");
  cls.def_property("cols",
                   &Class::cols,
                   &Class::set_cols,
                   "Image cols");
}

#endif //TASERV2_CAMERA_HELP_H
