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

template<typename T>
class _UnitQuaternionValidator {
 public:
  static void Validate(const Eigen::Quaternion<T> &cp) {
    if (!math::IsUnitQuaternion(cp)) {
      throw std::domain_error("Control point must be unit quaternion!");
    }
  };
};

template <typename T>
class UniformSO3SplineSegmentView : public SplineSegmentViewBase<T, Eigen::Quaternion<T>, 4, _UnitQuaternionValidator<T>> {
  using Quaternion = Eigen::Quaternion<T>;
  using QuaternionMap = Eigen::Map<Quaternion>;
  using Result = std::unique_ptr<TrajectoryEvaluation<T>>;
  using Vector4 = Eigen::Matrix<T, 4, 1>;
  using BaseType = SplineSegmentViewBase<T, Eigen::Quaternion<T>, 4, _UnitQuaternionValidator<T>>;
 public:
  using BaseType::Meta;

  // Import constructor
  using BaseType::SplineSegmentViewBase;

  Result Evaluate(T t, int flags) const override {
    auto result = std::make_unique<TrajectoryEvaluation<T>>();

    if (flags & EvalPosition)
      result->position.setZero();
    if (flags & EvalVelocity)
      result->velocity.setZero();
    if (flags & EvalAcceleration)
      result->acceleration.setZero();

    const bool do_angular_velocity = flags & EvalAngularVelocity;
    const bool do_orientation = flags & EvalOrientation;

    // Early return if we shouldn't calculate rotation components
    if (!(do_orientation || do_angular_velocity)) {
      return result;
    }

    // Since angular velocity computations implicitly requires orientation computations
    // we can henceforth assume do_orientation=true

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
    T u2 = ceres::pow(u, 2);
    T u3 = ceres::pow(u, 3);
    T dt_inv = T(1) / this->dt();

    U = Vector4(T(1), u, u2, u3);
    B = U.transpose() * M_cumul.cast<T>();

    if (do_angular_velocity) {
      dU = dt_inv * Vector4(T(0), T(1), T(2) * u, T(3) * u2);
      dB = dU.transpose() * M_cumul.cast<T>();
    }

    Quaternion &q = result->orientation;

    // These parts are updated when computing the derivative
    Quaternion dq_parts[3] = {{T(1), T(0), T(0), T(0)},
                              {T(1), T(0), T(0), T(0)},
                              {T(1), T(0), T(0), T(0)}};

    q = this->ControlPoint(i0);
    const int K = (i0 + 4);
    for (int i=(i0 + 1); i < K; ++i) {
      // Orientation
      QuaternionMap qa = this->ControlPoint(i - 1);
      QuaternionMap qb = this->ControlPoint(i);
      Quaternion omega = math::logq(qa.conjugate() * qb);
      Quaternion eomegab = math::expq(Quaternion(omega.coeffs() * B(i-i0)));
      q *= eomegab;

      // Angular velocity
      // This iterative scheme follows from the product rule of derivatives
      if (do_angular_velocity) {
        for (int j = (i0 + 1); j < K; ++j) {
          const int m = j - i0 - 1;
          if (i==j) {
            dq_parts[m] *= Quaternion(omega.coeffs()*dB(i-i0));
          }
          dq_parts[m] *= eomegab;
        }
      }
    }

    if (do_angular_velocity) {
      Quaternion dq = this->ControlPoint(i0) * Quaternion(dq_parts[0].coeffs() + dq_parts[1].coeffs() + dq_parts[2].coeffs());
      result->angular_velocity = math::angular_velocity(q, dq);
    }

    return result;
  }

};

template<typename T>
class UniformSO3SplineView : public SplineViewBase<T, UniformSO3SplineSegmentView> {
 public:
  // Inherit constructor
  using SplineViewBase<T, UniformSO3SplineSegmentView>::SplineViewBase;

};

} // namespace detail

class UniformSO3SplineTrajectory : public detail::SplinedTrajectoryBase<detail::UniformSO3SplineView> {
 public:
  static constexpr const char* CLASS_ID = "UniformSO3Spline";
  using SplinedTrajectoryBase<detail::UniformSO3SplineView>::SplinedTrajectoryBase;
};

} // namespace trajectories
} // namespace taser

#endif //TASERV2_UNIFORM_SO3_SPLINE_TRAJECTORY_H
