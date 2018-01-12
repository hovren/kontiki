#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include <Eigen/Dense>

#include "trajectories/uniform_r3_spline_trajectory.h"
#include "trajectory_helper.h"

namespace py = pybind11;

namespace TT = taser::trajectories;

PYBIND11_MODULE(_uniform_r3_spline_trajectory, m) {
  m.doc() = "Uniform splined trajectory in R3";

  using Class = TT::UniformR3SplineTrajectory;
  auto cls = py::class_<Class, std::shared_ptr<Class>>(m, "UniformR3SplineTrajectory");

  cls.def(py::init<>());
  cls.def_property_readonly("dt", &Class::dt);
  cls.def_property_readonly("t0", &Class::t0);
  cls.def("__len__", &Class::NumKnots);

  // Common trajectory methods/properties/...
  declare_trajectory_common<Class>(cls);

} // PYBIND11_MODULE