//
// Created by hannes on 2017-12-07.
//

#ifndef KONTIKIV2_TRAJECTORY_HELPER_H
#define KONTIKIV2_TRAJECTORY_HELPER_H

#include <kontiki/trajectories/trajectory.h>
namespace TT = kontiki::trajectories;

template<typename TrajectoryModel, typename PyClass>
void declare_trajectory_common(PyClass &cls) {
  using Class = TrajectoryModel;
  cls.def("position", &Class::Position, "Position in the world coorindate frame");
  cls.def("velocity", &Class::Velocity, "Velocity in the world coordinate frame");
  cls.def("acceleration", &Class::Acceleration, "Acceleration in the world coordinate frame");
  cls.def("orientation", [](Class &self, double t){
    Eigen::Quaterniond q = self.Orientation(t);
    Eigen::Vector4d out(q.w(), q.x(), q.y(), q.z());
    return out;
  },
  "Rotation that moves points from the trajectory to the world coordinate frame");
  cls.def("angular_velocity", &Class::AngularVelocity, "Angular velocity in the world coordinate frame");
  cls.def("from_world", &Class::FromWorld, "Move point from the world to the trajectory coordinate frame");
  cls.def("to_world", &Class::ToWorld, "Move point from the trajectory to the world coordinate frame");
  cls.def_property_readonly("min_time", &Class::MinTime, "Minimum time trajectory is valid for (including)");
  cls.def_property_readonly("max_time", &Class::MaxTime, "Maximum time the trajectory is valid for (excluding)");
  cls.def_property_readonly("valid_time", &Class::ValidTime, "Min and max time");
  cls.def_property("locked", &Class::IsLocked, &Class::Lock, "If True, trajectory will be kept constant during estimation");
  cls.def("clone", [](const Class &self){
    auto cloned = std::make_shared<Class>(self);
    return cloned;
  }, "Clone trajectory");
};

#endif //KONTIKIV2_TRAJECTORY_HELPER_H
