#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <Eigen/Dense>

#include <kontiki/trajectories/split_trajectory.h>
#include <kontiki/trajectories/uniform_r3_spline_trajectory.h>
#include <kontiki/trajectories/uniform_so3_spline_trajectory.h>
#include "trajectory_helper.h"

namespace py = pybind11;
using namespace pybind11::literals;

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

  cls.doc() = R"pbdoc(
  Split interpolation trajectory

  This trajectory is implemented using two splines: one in R3 and one in SO3.
  )pbdoc";

  cls.def(py::init<>());
  cls.def(py::init<std::shared_ptr<TT::UniformR3SplineTrajectory>,
                   std::shared_ptr<TT::UniformSO3SplineTrajectory>>());
  cls.def(py::init<double, double>());
  cls.def(py::init<double, double, double, double>(),
  "r3_dt"_a=1, "so3_dt"_a=1, "r3_t0"_a=0, "so3_t0"_a=0, "Create new trajectory");


  cls.def_property_readonly("R3_spline", &Class::R3Spline,
                            "The :class:`.UniformR3SplineTrajectory` instance");
  cls.def_property_readonly("SO3_spline", &Class::SO3Spline,
                            "The :class:`.UniformSO3SplineTrajectory` instance");

  // Common trajectory methods/properties/...
  declare_trajectory_common<Class>(cls);

} // PYBIND11_MODULE