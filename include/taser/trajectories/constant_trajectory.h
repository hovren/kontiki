//
// Created by hannes on 2017-11-29.
//

#ifndef TASERV2_CONSTANT_TRAJECTORY_H
#define TASERV2_CONSTANT_TRAJECTORY_H

#include "trajectory.h"
#include "trajectory_estimator.h"

#include <Eigen/Dense>

struct _ConstantMeta {

};

namespace taser {
namespace trajectories {



template<typename T>
class ConstantTrajectory : public TrajectoryBase<T, ConstantTrajectory<T>> {
  using Vector3 = Eigen::Matrix<T, 3, 1>;

 public:
  using Meta = _ConstantMeta;
  static constexpr const char* CLASS_ID = "Constant";

  ConstantTrajectory() : constant_(1, 2, 3) {};
  ConstantTrajectory(const Vector3& k) : constant_(k) {};

  Vector3 constant() const {
    return constant_;
  }

  void set_constant(const Vector3 &k) {
    constant_ = k;
  }

  std::unique_ptr<TrajectoryEvaluation<T>> Evaluate(const T t, const int flags) const {
    auto result = std::make_unique<TrajectoryEvaluation<T>>();
    if (!flags)
      throw std::logic_error("Evaluate() called with flags=0");
    if (flags & EvalPosition)
      result->position = constant_;
    if (flags & EvalVelocity)
      result->velocity.setZero();
    if (flags & EvalAcceleration)
      result->acceleration.setZero();
    if (flags & EvalOrientation)
      result->orientation.setIdentity();
    if (flags & EvalAngularVelocity)
      result->angular_velocity.setZero();
    return result;
  }

  // Add to problem, fill Meta struct, return parameter blocks
  void AddToEstimator(TrajectoryEstimator<ConstantTrajectory> &estimator, Meta& meta,
                      std::vector<double*> &parameter_blocks,
                      std::vector<size_t> &parameter_sizes) {
    // Fill meta
    // ... nothing to do

    // Define/add parameter blocks and add to problem
    // In this case we have only one: the constant slope parameter
    estimator.problem().AddParameterBlock(constant_.data(), 3);
    parameter_blocks.push_back(constant_.data());
    parameter_sizes.push_back(3);
  }

  static ConstantTrajectory<T>
  Unpack(T const* const* params, const Meta& meta) {
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> constant(&params[0][0]);
    ConstantTrajectory<T> trajectory(constant);
    return trajectory;
  }

 protected:
  Vector3 constant_;
};

} // namespace trajectories
} // namespace taser

#endif //TASERV2_CONSTANT_TRAJECTORY_H
