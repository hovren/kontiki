#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include <Eigen/Dense>
#include <trajectory_estimator.h>

#include "trajectories/constant_trajectory.h"

namespace py = pybind11;

template<template<typename> typename TrajectoryModel>
void declare_estimator(py::module &m) {
  using TrajectoryImpl = TrajectoryModel<double>;
  using Class = taser::TrajectoryEstimator<TrajectoryModel>;

  auto cls = py::class_<Class>(m, "TrajectoryEstimator");

  cls.def(py::init<std::shared_ptr<TrajectoryImpl>>());
  cls.def_property_readonly("trajectory", &Class::trajectory, "Get the trajectory");

}

PYBIND11_MODULE(_trajectory_estimator, m) {
  m.doc() = "Trajectory estimation class";

  py::module::import("taser.trajectories._constant_trajectory");

  using namespace taser::trajectories;
  declare_estimator<ConstantTrajectory>(m);
}