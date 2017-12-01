//
// Created by hannes on 2017-11-29.
//

#ifndef TASERV2_LINEAR_TRAJECTORY_H
#define TASERV2_LINEAR_TRAJECTORY_H

#include "trajectory.h"
#include "trajectory_estimator.h"

#include <Eigen/Dense>
#include <ceres/ceres.h>

struct _Meta {
  double t0;
};

namespace taser {
namespace trajectories {

template<typename T>
class LinearTrajectory : public TrajectoryBase<T, LinearTrajectory<T>> {
  using Vector3 = Eigen::Matrix<T, 3, 1>;
 public:
  using Meta = _Meta;
  static constexpr const char* CLASS_ID = "Linear";

  LinearTrajectory(double t0, const Vector3& k) : t0_(t0), constant_(k) {};

  Vector3 constant() const {
    return constant_;
  }

  void set_constant(const Vector3 &k) {
    constant_ = k;
  }

  Vector3 position_impl(T t) const {
    return constant_ * (t - T(t0_));
  }

  static LinearTrajectory<T>
  Unpack(T const* const* params, const Meta& meta) {
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> constant(&params[0][0]);
    LinearTrajectory<T> trajectory(meta.t0, constant);
    return trajectory;
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

};

} // namespace trajectories
} // namespace taser

#endif //TASERV2_LINEAR_TRAJECTORY_H
