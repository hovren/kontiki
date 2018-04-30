#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <Eigen/Dense>

#include <kontiki/trajectories/split_trajectory.h>
#include <kontiki/trajectories/uniform_r3_spline_trajectory.h>
#include <kontiki/trajectories/uniform_so3_spline_trajectory.h>
#include "trajectory_helper.h"

namespace py = pybind11;

namespace TT = kontiki::trajectories;

static constexpr int python_index_to_linear(int i, int N) {
  if ((i >= N) || (i < -N)) {
    throw py::index_error("Invalid sequence index");
  }
  else if ( i < 0 )
    return i + N;
  else
    return i;
}

PYBIND11_MODULE(_split_trajectory, m) {
  m.doc() = "Split R3+SO3 trajectory";

  using Class = TT::SplitTrajectory;
  auto cls = py::class_<Class, std::shared_ptr<Class>>(m, "SplitTrajectory");

  cls.def(py::init<>());
  cls.def(py::init<double, double>());
  cls.def(py::init<double, double, double, double>());
  cls.def(py::init<std::shared_ptr<TT::UniformR3SplineTrajectory>,
                   std::shared_ptr<TT::UniformSO3SplineTrajectory>>());

  cls.def_property_readonly("R3_spline", &Class::R3Spline);
  cls.def_property_readonly("SO3_spline", &Class::SO3Spline);

  // Common trajectory methods/properties/...
  declare_trajectory_common<Class>(cls);

} // PYBIND11_MODULE