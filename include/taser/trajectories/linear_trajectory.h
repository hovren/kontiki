//
// Created by hannes on 2017-11-29.
//

#ifndef TASERV2_LINEAR_TRAJECTORY_H
#define TASERV2_LINEAR_TRAJECTORY_H

#include "trajectory.h"
#include "trajectory_estimator.h"

#include <Eigen/Dense>
#include <ceres/ceres.h>

struct _LinearMeta : public taser::trajectories::MetaBase {
  _LinearMeta() {};
  _LinearMeta(double t0) : t0(t0) {};

  double t0;

  int num_parameters() const override {
    return 0;
  }
};

namespace taser {
namespace trajectories {

class LinearTrajectory {
  using Vector3 = Eigen::Vector3d;
 public:
  using Meta = _LinearMeta;
  static constexpr const char* CLASS_ID = "Linear";
  LinearTrajectory() : t0_(0), constant_(1, 1, 1) {};
  LinearTrajectory(double t0, const Vector3& k) : t0_(t0), constant_(k) {};

  Vector3 constant() const {
    return constant_;
  }

  void set_constant(const Vector3 &k) {
    constant_ = k;
  }

  double t0() const {
    return t0_;
  }

  void set_t0(double x) {
    t0_ = x;
  }

  template<typename T>
  class View {
    using Vector3 = Eigen::Matrix<T, 3, 1>;
    using Result = std::unique_ptr<TrajectoryEvaluation<T>>;
   public:
    View(T const* const* params, const Meta &meta) : params_(params), meta_(meta) { };

    const Vector3& constant() const {
      const Vector3 v = Eigen::Map<const Vector3>(params_[0]);
    }

    std::unique_ptr<TrajectoryEvaluation<T>> Evaluate(const T t, const int flags) const {
      auto result = std::make_unique<TrajectoryEvaluation<T>>();
      if (!flags)
        throw std::logic_error("Evaluate() called with flags=0");
      if (flags & EvalPosition)
        result->position = constant() * (t - T(meta_.t0));
      if (flags & EvalVelocity)
        result->velocity = constant();
      if (flags & EvalAcceleration)
        result->acceleration.setZero();
      if (flags & EvalOrientation)
        result->orientation = this->calculate_orientation(t);
      if (flags & EvalAngularVelocity)
        result->angular_velocity = constant();
      return result;
    }

    // FIXME: Belongs in trajectory.h base class
    Vector3 Position(const T t) const {
      Result result = this->Evaluate(t, EvalPosition);
      return result->position;
    }

   protected:
    T const* const* params_;
    const Meta &meta_;

    Eigen::Quaternion<T> calculate_orientation(T t) const {
      Vector3 c = constant();
      T theta = c.norm() * (t - meta_.t0);
      Vector3 n = c.normalized();
      Eigen::AngleAxis<T> aa(theta, n);
      return Eigen::Quaternion<T>(aa);
    }
  }; // ::View

  std::unique_ptr<TrajectoryEvaluation<double>> Evaluate(const double t, const int flags) const {
    Meta meta(t0_);
    const double *c = constant_.data();
    return View<double>(&c, meta).Evaluate(t, flags);
  }


  // Add to problem, fill Meta struct, return parameter blocks
  void AddToEstimator(TrajectoryEstimator<LinearTrajectory> &estimator,
                      const time_init_t &times,
                      Meta& meta,
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
  Vector3 constant_; // <-- The data to optimize

};

} // namespace trajectories
} // namespace taser

#endif //TASERV2_LINEAR_TRAJECTORY_H
