#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <Eigen/Dense>

#include "trajectories/uniform_r3_spline_trajectory.h"
#include "trajectory_helper.h"

namespace py = pybind11;

namespace TT = taser::trajectories;

static constexpr int python_index_to_linear(int i, int N) {
  if ((i >= N) || (i < -N)) {
    throw py::index_error("Invalid sequence index");
  }
  else if ( i < 0 )
    return i + N;
  else
    return i;
}

PYBIND11_MODULE(_uniform_r3_spline_trajectory, m) {
  m.doc() = "Uniform splined trajectory in R3";

  using Class = TT::UniformR3SplineTrajectory;
  auto cls = py::class_<Class, std::shared_ptr<Class>>(m, "UniformR3SplineTrajectory");

  cls.def(py::init<>());
  cls.def(py::init<double>());
  cls.def(py::init<double, double>());
  cls.def_property_readonly("dt", &Class::dt);
  cls.def_property_readonly("t0", &Class::t0);
  cls.def("__len__", &Class::NumKnots);
  cls.def("__getitem__", [&](Class& self, int i) {
    Eigen::Vector3d cp = self.ControlPoint(python_index_to_linear(i, self.NumKnots()));
    return cp;
  });
  cls.def("__setitem__", [&](Class& self, int i, Eigen::Vector3d cp) {
    self.ControlPoint(python_index_to_linear(i, self.NumKnots())) = cp;
  });
  cls.def("append_knot", &Class::AppendKnot);

  // Common trajectory methods/properties/...
  declare_trajectory_common<Class>(cls);

} // PYBIND11_MODULE