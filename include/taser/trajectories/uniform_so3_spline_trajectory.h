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

    q = ControlPoint(i0);
    const int K = (i0 + 4);
    for (int i=(i0 + 1); i < K; ++i) {
      // Orientation
      QuaternionMap qa = ControlPoint(i - 1);
      QuaternionMap qb = ControlPoint(i);
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
      Quaternion dq = ControlPoint(i0)*Quaternion(dq_parts[0].coeffs() + dq_parts[1].coeffs() + dq_parts[2].coeffs());
      result->angular_velocity = math::angular_velocity(q, dq);
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

    int i1, i2;
    double u_notused;
    double t1, t2;

    for (auto tt : times) {
      t1 = tt.first;
      t2 = tt.second;
    }

    // Find control point range
    AsView().CalculateIndexAndInterpolationAmount(t1, i1, u_notused);
    AsView().CalculateIndexAndInterpolationAmount(t2, i2, u_notused);

    for (int i=i1; i < i2 + 4; ++i) {
      auto ptr = this->holder_->Parameter(i);
      const int size = 4;
      parameter_blocks.push_back(ptr);
      parameter_sizes.push_back(size);
      problem.AddParameterBlock(ptr, size);
    }
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
