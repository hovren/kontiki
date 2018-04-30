#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <Eigen/Dense>

#include <kontiki/trajectories/uniform_r3_spline_trajectory.h>
#include "trajectory_helper.h"
#include "spline_helpers.h"

namespace py = pybind11;

namespace TT = kontiki::trajectories;

struct PyR3SplineHelper {
  using PyControlPointType = Eigen::Vector3d;

  static Eigen::Vector3d ConvertCppToPy(const Eigen::Vector3d& cp) {
    return cp;
  }

  static Eigen::Vector3d ConvertPyToCpp(const Eigen::Vector3d& cp) {
    return cp;
  }
};

PYBIND11_MODULE(_uniform_r3_spline_trajectory, m) {
  m.doc() = "Uniform splined trajectory in R3";

  using Class = TT::UniformR3SplineTrajectory;
  using Helper = PyR3SplineHelper;
  auto cls = py::class_<Class, std::shared_ptr<Class>>(m, "UniformR3SplineTrajectory");
  cls.doc() = R"pbdoc( A spline with control points in R3

  Control points are vectors in R3.
  )pbdoc";

  // Common attributes
  declare_spline_common<Class, Helper>(cls);
  declare_trajectory_common<Class>(cls);

} // PYBIND11_MODULE