#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <Eigen/Dense>

#include <kontiki/trajectories/uniform_so3_spline_trajectory.h>
#include "trajectory_helper.h"
#include "spline_helpers.h"

namespace py = pybind11;

namespace TT = kontiki::trajectories;

static Eigen::Quaterniond vec_to_quat(const Eigen::Vector4d& qv) {
  return Eigen::Quaterniond(qv(0), qv(1), qv(2), qv(3));
}

static Eigen::Vector4d quat_to_vec(const Eigen::Quaterniond& q) {
  return Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
}

struct PySO3SplineHelper {
  using PyControlPointType = Eigen::Vector4d;

  static Eigen::Vector4d ConvertCppToPy(const Eigen::Quaterniond& cp) {
    return quat_to_vec(cp);
  }

  static Eigen::Quaterniond ConvertPyToCpp(const Eigen::Vector4d& cp) {
    return vec_to_quat(cp);
  }
};


PYBIND11_MODULE(_uniform_so3_spline_trajectory, m) {
  m.doc() = "Uniform splined trajectory in SO3";

  using Class = TT::UniformSO3SplineTrajectory;
  using Helper = PySO3SplineHelper;
  auto cls = py::class_<Class, std::shared_ptr<Class>>(m, "UniformSO3SplineTrajectory");

  // Common attributes
  declare_spline_common<Class, Helper>(cls);
  declare_trajectory_common<Class>(cls);

} // PYBIND11_MODULE