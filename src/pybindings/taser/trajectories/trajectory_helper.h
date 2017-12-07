//
// Created by hannes on 2017-12-07.
//

#ifndef TASERV2_TRAJECTORY_HELPER_H
#define TASERV2_TRAJECTORY_HELPER_H

#include "trajectories/trajectory.h"
namespace TT = taser::trajectories;

template<template<typename> typename TrajectoryModel, typename PyClass>
void declare_trajectory_common(PyClass &cls) {
  using Class = TrajectoryModel<double>;
  cls.def("position", &TT::TrajectoryBase<double, TrajectoryModel<double>>::Position);
  cls.def("velocity", &TT::TrajectoryBase<double, TrajectoryModel<double>>::Velocity);
  cls.def("acceleration", &TT::TrajectoryBase<double, TrajectoryModel<double>>::Acceleration);
  cls.def("orientation", [](Class &self, double t){
    Eigen::Quaterniond q = self.Orientation(t);
    Eigen::Vector4d out(q.w(), q.x(), q.y(), q.z());
    return out;
  });
  cls.def("angular_velocity", &TT::TrajectoryBase<double, TrajectoryModel<double>>::AngularVelocity);
  cls.def("from_world", &TT::TrajectoryBase<double, TrajectoryModel<double>>::FromWorld);
  cls.def("to_world", &TT::TrajectoryBase<double, TrajectoryModel<double>>::ToWorld);
};

#endif //TASERV2_TRAJECTORY_HELPER_H
