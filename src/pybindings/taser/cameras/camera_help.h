//
// Created by hannes on 2017-12-04.
//

#ifndef TASERV2_CAMERA_HELP_H
#define TASERV2_CAMERA_HELP_H

#include "cameras/camera.h"

namespace py = pybind11;

namespace C = taser::cameras;

template<typename Class, typename PyClass>
void declare_common(PyClass &cls) {
  cls.def(py::init<int, int, double>());
  cls.def("project", &Class::template Project<double>, "Project");
  cls.def("unproject", &Class::template Unproject<double>, "Unproject");
  cls.def_property("readout",
                   &C::CameraBase::readout,
                   &C::CameraBase::set_readout,
                   "Rolling shutter readout time");
  cls.def_property("rows",
                   &C::CameraBase::rows,
                   &C::CameraBase::set_rows,
                   "Image rows");
  cls.def_property("cols",
                   &C::CameraBase::cols,
                   &C::CameraBase::set_cols,
                   "Image cols");
}

#endif //TASERV2_CAMERA_HELP_H
