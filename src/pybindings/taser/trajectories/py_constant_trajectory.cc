//
// Created by hannes on 2017-11-29.
//

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include <Eigen/Dense>

#include "trajectories/constant_trajectory.h"
#include "trajectory_helper.h"
namespace py = pybind11;

void declare_constant_trajectory(py::module &m) {
  using Class = taser::trajectories::ConstantTrajectory<double>;
  auto cls = py::class_<Class, std::shared_ptr<Class>>(m, "ConstantTrajectory");

  cls.def(py::init<const Eigen::Vector3d &>());
  cls.def_property("constant", &Class::constant, &Class::set_constant);

  // Common attributes
  declare_trajectory_common<TT::ConstantTrajectory>(cls);
}

PYBIND11_MODULE(_constant_trajectory, m) {
  m.doc() = "Constant Trajectory for testing purposes";

  using Class = taser::trajectories::ConstantTrajectory<double>;
  auto cls = py::class_<Class, std::shared_ptr<Class>>(m, "ConstantTrajectory");

  cls.def(py::init<const Eigen::Vector3d &>());
  cls.def_property("constant", &Class::constant, &Class::set_constant);

  // Common attributes
  declare_trajectory_common<TT::ConstantTrajectory>(cls);
} // PYBIND11_MODULE