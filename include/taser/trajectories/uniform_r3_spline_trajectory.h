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
class UniformR3SplineSegmentView : public SplineSegmentViewBase<T> {
  using Result = std::unique_ptr<TrajectoryEvaluation<T>>;
  using Vector3 = Eigen::Matrix<T, 3, 1>;
  using Vector4 = Eigen::Matrix<T, 4, 1>;
  using Vector3Map = Eigen::Map<Vector3>;
 public:
  using Meta = SplineMeta;

  // Import constructor
  using SplineSegmentViewBase<T>::SplineSegmentViewBase;

  const Vector3Map ControlPoint(int i) const {
    return Vector3Map(this->holder_->Parameter(i));
  }

  Vector3Map MutableControlPoint(int i) {
    return Vector3Map(this->holder_->Parameter(i));
  }

  Result Evaluate(T t, int flags) const override {
    auto result = std::make_unique<TrajectoryEvaluation<T>>();

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

    if ((flags & EvalPosition) || (flags & EvalVelocity))
      u2 = ceres::pow(u, 2);
    if (flags & EvalPosition)
      u3 = ceres::pow(u, 3);

    if (flags & EvalPosition) {
      Up = Vector4(T(1), u, u2, u3);
      Bp = Up.transpose() * M.cast<T>();
      p.setZero();
    }

    if (flags & EvalVelocity) {
      Uv = dt_inv * Vector4(T(0), T(1), T(2) * u, T(3) * u2);
      Bv = Uv.transpose() * M.cast<T>();
      v.setZero();
    }

    if (flags & EvalAcceleration) {
      Ua = ceres::pow(dt_inv, 2) *  Vector4(T(0), T(0), T(2), T(6) * u);
      Ba = Ua.transpose() * M.cast<T>();
      a.setZero();
    }


    for (int i=i0; i < i0 + 4; ++i) {
      Vector3Map cp = ControlPoint(i);

      if (flags & EvalPosition)
        p += Bp(i - i0) * cp;

      if (flags & EvalVelocity)
        v += Bv(i - i0) * cp;

      if (flags & EvalAcceleration)
        a += Ba(i - i0) * cp;

    }

    // This trajectory is not concerned with orientations, so just return identity/zero if requested
    if (flags & EvalOrientation)
      result->orientation.setIdentity();
    if (flags & EvalAngularVelocity)
      result->angular_velocity.setZero();

    return result;
  }
};

template<typename T>
class UniformR3SplineView : public SplineViewBase<T, detail::UniformR3SplineSegmentView> {
  using Vector3 = Eigen::Matrix<T, 3, 1>;
  using Vector3Map = Eigen::Map<Vector3>;
 public:
  using SplineViewBase<T, detail::UniformR3SplineSegmentView>::SplineViewBase;

  const Vector3Map ControlPoint(int i) const {
    return this->ConcreteSegmentViewOrError().ControlPoint(i);
  }

  Vector3Map MutableControlPoint(int i) {
    return this->ConcreteSegmentViewOrError().MutableControlPoint(i);
  }
};

} // namespace detail

class UniformR3SplineTrajectory : public detail::SplinedTrajectoryBase<detail::UniformR3SplineView> {
  using Vector3 = Eigen::Vector3d;
  using Vector3Map = Eigen::Map<Vector3>;
 public:
  static constexpr const char* CLASS_ID = "UniformR3Spline";
  using SplinedTrajectoryBase<detail::UniformR3SplineView>::SplinedTrajectoryBase;

  UniformR3SplineTrajectory(double dt, double t0) :
      SplinedTrajectoryBase() {
    this->meta_.segments.push_back(detail::SplineSegmentMeta(dt, t0));
  };

  UniformR3SplineTrajectory(double dt) :
      UniformR3SplineTrajectory(dt, 0.0) { };

  UniformR3SplineTrajectory() :
      UniformR3SplineTrajectory(1.0) { };

  Vector3Map ControlPoint(size_t i) {
    return AsView().MutableControlPoint(i);
  }

  void AppendKnot(const Vector3& cp) {
    auto i = this->holder_->AddParameter(3);
    AsView().MutableControlPoint(i) = cp;
    // FIXME: Should check for single segment or give error
    this->meta_.segments[0].n += 1;
  }
};

} // namespace trajectories
} // namespace taser

#endif //TASERV2_UNIFORM_R3_SPLINE_TRAJECTORY_H
