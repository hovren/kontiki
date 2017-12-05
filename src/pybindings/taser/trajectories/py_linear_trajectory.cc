//
// Created by hannes on 2017-11-29.
//

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include <Eigen/Dense>

#include "trajectories/linear_trajectory.h"

namespace py = pybind11;

namespace TT = taser::trajectories;

// FIXME: Move to helper include
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

void declare_linear_trajectory(py::module &m) {
  using Class = TT::LinearTrajectory<double>;
  auto cls = py::class_<Class, std::shared_ptr<Class>>(m, "LinearTrajectory");

  cls.def(py::init<double, const Eigen::Vector3d &>());
  cls.def_property("constant", &Class::constant, &Class::set_constant);

  // Common trajectory methods/properties/...
  declare_trajectory_common<TT::LinearTrajectory>(cls);
}

PYBIND11_MODULE(_linear_trajectory, m) {
  m.doc() = "Linear Trajectory for testing purposes";

  declare_linear_trajectory(m);

} // PYBIND11_MODULE