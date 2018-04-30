#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <Eigen/Dense>
#include <sophus/se3.hpp>

#include <kontiki/trajectories/uniform_se3_spline_trajectory.h>
#include "trajectory_helper.h"
#include "spline_helpers.h"

namespace py = pybind11;

namespace TT = kontiki::trajectories;


struct PySE3SplineHelper {
  using PyControlPointType = Eigen::Matrix4d;

  static Eigen::Matrix4d ConvertCppToPy(const Sophus::SE3d & cp) {
    return cp.matrix();
  }

  static Sophus::SE3d ConvertPyToCpp(const Eigen::Matrix4d& cp) {
    ValidateSE3Element(cp); // Throws domain_error on error
    return Sophus::SE3d(cp);
  }

  static void ValidateSE3Element(const Eigen::Matrix4d &cp) {
    double eps = Sophus::Constants<double>::epsilon();
    Eigen::Matrix3d R = cp.topLeftCorner(3, 3);
    if (std::abs<double>(R.determinant() - 1) >= eps) {
      throw std::domain_error("Rotation matrix determinant is not 1!");
    }
    else if ((cp.row(3) - Eigen::Matrix<double, 1, 4>(0, 0, 0, 1)).squaredNorm() >= eps) {
      throw std::domain_error("Final row must be [0, 0, 0, 1]");
    }
  }
};


PYBIND11_MODULE(_uniform_se3_spline_trajectory, m) {
  m.doc() = "Uniform splined trajectory in SE3";

  using Class = TT::UniformSE3SplineTrajectory;
  using Helper = PySE3SplineHelper;
  auto cls = py::class_<Class, std::shared_ptr<Class>>(m, "UniformSE3SplineTrajectory");

  cls.def("evaluate", [](Class &self, double t){
    Sophus::SE3d P_SE3;
    Eigen::Matrix4d P, P_prim, P_bis;
    self.EvaluateSpline(t, 0xff, P_SE3, P_prim, P_bis);
    P = P_SE3.matrix();

    return std::make_tuple(P, P_prim, P_bis);
  });


  // Common attributes
  declare_spline_common<Class, Helper>(cls);
  declare_trajectory_common<Class>(cls);

} // PYBIND11_MODULE