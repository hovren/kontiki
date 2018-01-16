#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <Eigen/Dense>

#include "trajectories/uniform_so3_spline_trajectory.h"
#include "trajectory_helper.h"

namespace py = pybind11;

namespace TT = taser::trajectories;

// FIXME: Move to shared file
static constexpr int python_index_to_linear(int i, int N) {
  if ((i >= N) || (i < -N)) {
    throw py::index_error("Invalid sequence index");
  }
  else if ( i < 0 )
    return i + N;
  else
    return i;
}

static Eigen::Quaterniond vec_to_quat(const Eigen::Vector4d& qv) {
  return Eigen::Quaterniond(qv(0), qv(1), qv(2), qv(3));
}

static Eigen::Vector4d quat_to_vec(const Eigen::Quaterniond& q) {
  return Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
}

PYBIND11_MODULE(_uniform_so3_spline_trajectory, m) {
  m.doc() = "Uniform splined trajectory in SO3";

  using Class = TT::UniformSO3SplineTrajectory;
  auto cls = py::class_<Class, std::shared_ptr<Class>>(m, "UniformSO3SplineTrajectory");

  cls.def(py::init<>());
  cls.def(py::init<double>());
  cls.def(py::init<double, double>());
  cls.def_property_readonly("dt", &Class::dt);
  cls.def_property_readonly("t0", &Class::t0);
  cls.def("__len__", &Class::NumKnots);
  cls.def("__getitem__", [&](Class& self, int i) {
    Eigen::Quaterniond cp = self.ControlPoint(python_index_to_linear(i, self.NumKnots()));
    return quat_to_vec(cp);
  });
  cls.def("__setitem__", [&](Class& self, int i, Eigen::Vector4d cp) {
    self.ControlPoint(python_index_to_linear(i, self.NumKnots())) = vec_to_quat(cp);
  });
  cls.def("append_knot", [&](Class& self, Eigen::Vector4d qvec) {
    self.AppendKnot(vec_to_quat(qvec));
  });

  // Common trajectory methods/properties/...
  declare_trajectory_common<Class>(cls);

} // PYBIND11_MODULE