//
// Created by hannes on 2017-11-29.
//

#ifndef TASERV2_LINEAR_TRAJECTORY_H
#define TASERV2_LINEAR_TRAJECTORY_H

#include "trajectory.h"
#include "trajectory_estimator.h"

#include <Eigen/Dense>
#include <ceres/ceres.h>

struct _LinearMeta {
  double t0;
};

namespace taser {
namespace trajectories {

template<typename T>
class LinearTrajectory : public TrajectoryBase<T, LinearTrajectory<T>> {
  using Vector3 = Eigen::Matrix<T, 3, 1>;
 public:
  using Meta = _LinearMeta;
  static constexpr const char* CLASS_ID = "Linear";
  LinearTrajectory() : t0_(0), constant_(1, 1, 1) {};
  LinearTrajectory(double t0, const Vector3& k) : t0_(t0), constant_(k) {};
  LinearTrajectory(T const* const* params, const Meta& meta) {
    constant_ = Eigen::Map<const Eigen::Matrix<T, 3, 1>>(&params[0][0]);
    t0_ = meta.t0;
  }

  Vector3 constant() const {
    return constant_;
  }

  void set_constant(const Vector3 &k) {
    constant_ = k;
  }

  T t0() const {
    return t0_;
  }

  void set_t0(T x) {
    t0_ = x;
  }

  std::unique_ptr<TrajectoryEvaluation<T>> Evaluate(const T t, const int flags) const {
    auto result = std::make_unique<TrajectoryEvaluation<T>>();
    if (!flags)
      throw std::logic_error("Evaluate() called with flags=0");
    if (flags & EvalPosition)
      result->position = constant_ * (t - T(t0_));
    if (flags & EvalVelocity)
      result->velocity = constant_;
    if (flags & EvalAcceleration)
      result->acceleration.setZero();
    if (flags & EvalOrientation)
      result->orientation = this->calculate_orientation(t);
    if (flags & EvalAngularVelocity)
      result->angular_velocity = constant_;
    return result;
  }

  // Add to problem, fill Meta struct, return parameter blocks
  void AddToEstimator(TrajectoryEstimator<LinearTrajectory> &estimator, Meta& meta,
                    std::vector<double*> &parameter_blocks,
                    std::vector<size_t> &parameter_sizes) {
    // Fill meta
    meta.t0 = t0_;

    // Define/add parameter blocks and add to problem
    // In this case we have only one: the constant slope parameter
    estimator.problem().AddParameterBlock(constant_.data(), 3);
    parameter_blocks.push_back(constant_.data());
    parameter_sizes.push_back(3);
  }

 protected:
  double t0_;
  Vector3 constant_;

  Eigen::Quaternion<T> calculate_orientation(T t) const {
    T theta = constant_.norm() * (t - t0_);
    Vector3 n = constant_.normalized();
    Eigen::AngleAxis<T> aa(theta, n);
    return Eigen::Quaternion<T>(aa);
  }

};

} // namespace trajectories
} // namespace taser

#endif //TASERV2_LINEAR_TRAJECTORY_H
