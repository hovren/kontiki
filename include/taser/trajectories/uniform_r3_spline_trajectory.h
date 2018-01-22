#ifndef TASERV2_UNIFORM_R3_SPLINE_TRAJECTORY_H
#define TASERV2_UNIFORM_R3_SPLINE_TRAJECTORY_H

#include <vector>
#include <cmath>

#include <Eigen/Dense>

#include "trajectory.h"
#include "spline_base.h"
#include "../trajectory_estimator.h"
#include "dataholders/vectorholder.h"

namespace taser {
namespace trajectories {

namespace detail {

template<typename T>
class UniformR3SplineSegmentView : public SplineSegmentViewBase<T, Eigen::Matrix<T, 3, 1>, 3> {
  using Result = std::unique_ptr<TrajectoryEvaluation<T>>;
  using Vector3 = Eigen::Matrix<T, 3, 1>;
  using Vector4 = Eigen::Matrix<T, 4, 1>;
  using Vector3Map = Eigen::Map<Vector3>;
  using BaseType = SplineSegmentViewBase<T, Eigen::Matrix<T, 3, 1>, 3>;
 public:
  using Meta = SplineMeta;

  // Import constructor
  using BaseType::SplineSegmentViewBase;

  Result Evaluate(T t, int flags) const override {
    auto result = std::make_unique<TrajectoryEvaluation<T>>(flags);

    int i0;
    T u;
    this->CalculateIndexAndInterpolationAmount(t, i0, u);
//    std::cout << "t=" << t << " i0=" << i0 << " u=" << u << std::endl;

    const size_t N = this->NumKnots();
    if ((N < 4) || (i0 < 0) || (i0 > (N - 4))) {
      std::stringstream ss;
      ss << "t=" << t << " i0=" << i0 << " is out of range for spline with ncp=" << N;
      throw std::range_error(ss.str());
    }

    Vector4 Up, Uv, Ua;
    Vector4 Bp, Bv, Ba;
    T u2, u3;
    Vector3 &p = result->position;
    Vector3 &v = result->velocity;
    Vector3 &a = result->acceleration;
    T dt_inv = T(1) / this->dt();

    if (result->needs.Position() || result->needs.Velocity())
      u2 = ceres::pow(u, 2);
    if (flags & EvalPosition)
      u3 = ceres::pow(u, 3);

    if (result->needs.Position()) {
      Up = Vector4(T(1), u, u2, u3);
      Bp = Up.transpose() * M.cast<T>();
      p.setZero();
    }

    if (result->needs.Velocity()) {
      Uv = dt_inv * Vector4(T(0), T(1), T(2) * u, T(3) * u2);
      Bv = Uv.transpose() * M.cast<T>();
      v.setZero();
    }

    if (result->needs.Acceleration()) {
      Ua = ceres::pow(dt_inv, 2) *  Vector4(T(0), T(0), T(2), T(6) * u);
      Ba = Ua.transpose() * M.cast<T>();
      a.setZero();
    }


    for (int i=i0; i < i0 + 4; ++i) {
      Vector3Map cp = this->ControlPoint(i);

      if (flags & EvalPosition)
        p += Bp(i - i0) * cp;

      if (flags & EvalVelocity)
        v += Bv(i - i0) * cp;

      if (flags & EvalAcceleration)
        a += Ba(i - i0) * cp;

    }

    // This trajectory is not concerned with orientations, so just return identity/zero if requested
    if (result->needs.Orientation())
      result->orientation.setIdentity();
    if (result->needs.AngularVelocity())
      result->angular_velocity.setZero();

    return result;
  }
};

template<typename T>
class UniformR3SplineView : public SplineViewBase<T, detail::UniformR3SplineSegmentView> {
 public:
  // Inherit constructor
  using SplineViewBase<T, detail::UniformR3SplineSegmentView>::SplineViewBase;
};

} // namespace detail

class UniformR3SplineTrajectory : public detail::SplinedTrajectoryBase<detail::UniformR3SplineView> {
 public:
  static constexpr const char* CLASS_ID = "UniformR3Spline";
  using SplinedTrajectoryBase<detail::UniformR3SplineView>::SplinedTrajectoryBase;

};

} // namespace trajectories
} // namespace taser

#endif //TASERV2_UNIFORM_R3_SPLINE_TRAJECTORY_H
