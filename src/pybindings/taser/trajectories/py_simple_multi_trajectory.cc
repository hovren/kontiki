//
// Created by hannes on 2017-11-29.
//

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <Eigen/Dense>

#include "trajectories/simple_multi_trajectory.h"
#include "trajectory_helper.h"

namespace py = pybind11;

namespace TT = taser::trajectories;

void declare_simple_trajectory(py::module &m) {
  using Class = TT::SimpleMultiTrajectory;
  auto cls = py::class_<Class, std::shared_ptr<Class>>(m, "SimpleMultiTrajectory");

  cls.def(py::init<>());
  cls.def_readonly("foo_a", &Class::foo_a);
  cls.def_readonly("foo_b", &Class::foo_b);


  // Common trajectory methods/properties/...
  declare_trajectory_common<Class>(cls);
}

void declare_foo_trajectory(py::module &m) {
  using Class = TT::FooTrajectory;
  auto cls = py::class_<Class, std::shared_ptr<Class>>(m, "FooTrajectory");

  cls.def(py::init<>());

  cls.def("add_vector", &Class::AddVector);
  cls.def_property("foo", &Class::foo, &Class::set_foo);
  cls.def_property_readonly("vectors", &Class::vectors);

  // Common trajectory methods/properties/...
  declare_trajectory_common<Class>(cls);
}

PYBIND11_MODULE(_simple_multi_trajectory, m) {
  m.doc() = "Linear Trajectory for testing purposes";

  declare_foo_trajectory(m);
  declare_simple_trajectory(m);

} // PYBIND11_MODULE