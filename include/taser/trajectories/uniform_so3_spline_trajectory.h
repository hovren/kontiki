//
// Created by hannes on 2018-01-15.
//

#ifndef TASERV2_UNIFORM_SO3_SPLINE_TRAJECTORY_H
#define TASERV2_UNIFORM_SO3_SPLINE_TRAJECTORY_H

#include <Eigen/Dense>

#include "trajectory.h"
#include "spline_base.h"
#include "../math/quaternion_math.h"

namespace taser {
namespace trajectories {
namespace detail {

template <typename T>
class UniformSO3SplineView : public SplineViewBase<T> {
  using Quaternion = Eigen::Quaternion<T>;
  using QuaternionMap = Eigen::Map<Quaternion>;
  using Result = std::unique_ptr<TrajectoryEvaluation<T>>;
  using Vector3 = Eigen::Matrix<T, 3, 1>;
  using Vector4 = Eigen::Matrix<T, 4, 1>;
 public:
  using SplineViewBase<T>::Meta;
  // Import constructor
  using SplineViewBase<T>::SplineViewBase;

  const QuaternionMap ControlPoint(int i) const {
    return QuaternionMap(this->holder_->Parameter(i));
  }

  QuaternionMap MutableControlPoint(int i) const {
    return QuaternionMap(this->holder_->Parameter(i));
  }

  Result Evaluate(T t, int flags) const override {
    auto result = std::make_unique<TrajectoryEvaluation<T>>();

    int i0;
    T u;
    this->CalculateIndexAndInterpolationAmount(t, i0, u);

    const size_t N = this->NumKnots();
    if ((N < 4) || (i0 < 0) || (i0 > (N - 4))) {
      std::stringstream ss;
      ss << "t=" << t << " i0=" << i0 << " is out of range for spline with ncp=" << N;
      throw std::range_error(ss.str());
    }

    Vector4 U, dU;
    Vector4 B, dB;
    T u2, u3;

    u2 = ceres::pow(u, 2);
    u3 = ceres::pow(u, 3);

    U = Vector4(T(1), u, u2, u3);
    B = U.transpose() * M_cumul.cast<T>();

    Quaternion &q = result->orientation;

    q = ControlPoint(i0);
    for (int i=(i0 + 1); i < (i0 + 4); ++i) {
      QuaternionMap qa = ControlPoint(i - 1);
      QuaternionMap qb = ControlPoint(i);
      Quaternion omega = math::logq(qa.conjugate() * qb);
      Quaternion eomegab = math::expq(Quaternion(omega.coeffs() * B(i)));
      q *= eomegab;
    }

    return result;
  }

};

} // namespace detail

class UniformSO3SplineTrajectory : public detail::SplinedTrajectoryBase<detail::UniformSO3SplineView> {
  using Quaternion = Eigen::Quaterniond;
  using QuaternionMap = Eigen::Map<Quaternion>;
 public:
  static constexpr const char* CLASS_ID = "UniformSO3Spline";
  using SplinedTrajectoryBase<detail::UniformSO3SplineView>::SplinedTrajectoryBase;

  QuaternionMap ControlPoint(size_t i) {
    return AsView().MutableControlPoint(i);
  }

  // FIXME: Should be called AppendControlPoint
  void AppendKnot(const Quaternion& cp) {
    if (IsUnitQuaternion(cp)) {
      auto i = this->holder_->AddParameter(4);
      AsView().MutableControlPoint(i) = cp;
      this->meta_.n += 1;
    }
    else {
      throw std::domain_error("Control points must be unit quaternions");
    }
  }

  void AddToProblem(ceres::Problem& problem,
                    const time_init_t &times,
                    Meta& meta,
                    std::vector<double*> &parameter_blocks,
                    std::vector<size_t> &parameter_sizes) const override {

  }

 protected:
  bool IsUnitQuaternion(const Quaternion& q) {
    auto err = std::abs(q.norm() - 1);
    return err < math::eps_unit_check;
  }

};

} // namespace trajectories
} // namespace taser

#endif //TASERV2_UNIFORM_SO3_SPLINE_TRAJECTORY_H
