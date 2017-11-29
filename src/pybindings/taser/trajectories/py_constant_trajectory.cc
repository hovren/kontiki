//
// Created by hannes on 2017-11-29.
//

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include <Eigen/Dense>

#include "trajectories/constant_trajectory.h"

namespace py = pybind11;

void declare_constant_trajectory(py::module &m) {
  using Class = taser::trajectories::ConstantTrajectory<double>;
  auto cls = py::class_<Class, std::shared_ptr<Class>>(m, "ConstantTrajectory");

  cls.def(py::init<const Eigen::Vector3d &>());
  cls.def_property("constant", &Class::constant, &Class::set_constant);
}

PYBIND11_MODULE(_constant_trajectory, m) {
  m.doc() = "Constant Trajectory for testing purposes";

  m.def("testing", []{
    return 6;
  });

  declare_constant_trajectory(m);

} // PYBIND11_MODULE