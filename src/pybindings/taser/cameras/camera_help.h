//
// Created by hannes on 2017-12-04.
//

#ifndef TASERV2_CAMERA_HELP_H
#define TASERV2_CAMERA_HELP_H

#include "cameras/camera.h"

#include <pybind11/eigen.h>
#include <pybind11/stl.h>

namespace py = pybind11;

namespace C = taser::cameras;

// Eigen::Quaternion not exposed, to use Vector4d instead
using PyRelPosePair = std::pair<Eigen::Vector4d, Eigen::Vector3d>;

template<typename Class, typename PyClass>
void declare_common(PyClass &cls) {


  cls.def(py::init<int, int, double>());
  cls.def("project", &Class::template Project<double>, "Project");
  cls.def("unproject", &Class::template Unproject<double>, "Unproject");
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
  cls.def_property("relative_pose", [](Class &self) {
    const C::RelativePose &pose_struct = self.relative_pose();
    const Eigen::Quaterniond &q = pose_struct.orientation;
    Eigen::Vector4d pyq(q.w(), q.x(), q.y(), q.z());
    return PyRelPosePair(pyq, pose_struct.translation);
  },
  [](Class &self, const PyRelPosePair &pose) {
    //Eigen::Quaterniond q(pose.first.data());
    const Eigen::Vector4d &q = pose.first;
   self.set_relative_pose(C::RelativePose(Eigen::Quaterniond(q(0), q(1), q(2), q(3)),
                                          pose.second));
  });

  cls.def("from_trajectory", &Class::template FromTrajectory<double>);
  cls.def("to_trajectory", &Class::template ToTrajectory<double>);
}

#endif //TASERV2_CAMERA_HELP_H
