//
// Created by hannes on 2017-12-07.
//

#ifndef TASERV2_TRAJECTORY_HELPER_H
#define TASERV2_TRAJECTORY_HELPER_H

#include "trajectories/trajectory.h"
namespace TT = taser::trajectories;

template<typename TrajectoryModel, typename PyClass>
void declare_trajectory_common(PyClass &cls) {
  using Class = TrajectoryModel;
  cls.def("position", &Class::Position);
  cls.def("velocity", &Class::Velocity);
  cls.def("acceleration", &Class::Acceleration);
  cls.def("orientation", [](Class &self, double t){
    Eigen::Quaterniond q = self.Orientation(t);
    Eigen::Vector4d out(q.w(), q.x(), q.y(), q.z());
    return out;
  });
  cls.def("angular_velocity", &Class::AngularVelocity);
  cls.def("from_world", &Class::FromWorld);
  cls.def("to_world", &Class::ToWorld);
  cls.def_property_readonly("min_time", &Class::MinTime);
  cls.def_property_readonly("max_time", &Class::MaxTime);
  cls.def_property_readonly("valid_time", &Class::ValidTime);
  cls.def_property("locked", &Class::IsLocked, &Class::Lock);
};

#endif //TASERV2_TRAJECTORY_HELPER_H
