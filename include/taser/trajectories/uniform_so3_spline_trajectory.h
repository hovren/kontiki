//
// Created by hannes on 2018-01-15.
//

#ifndef TASERV2_UNIFORM_SO3_SPLINE_TRAJECTORY_H
#define TASERV2_UNIFORM_SO3_SPLINE_TRAJECTORY_H

#include <Eigen/Dense>

#include "trajectory.h"
#include "spline_base.h"

namespace taser {
namespace trajectories {
namespace detail {

template <typename T>
class UniformSO3SplineView : public SplineViewBase<T> {
  using Quaternion = Eigen::Quaternion<T>;
  using QuaternionMap = Eigen::Map<Quaternion>;
  using Result = std::unique_ptr<TrajectoryEvaluation<T>>;
 public:
  using SplineViewBase<T>::Meta;
  // Import constructor
  using SplineViewBase<T>::SplineViewBase;

  Result Evaluate(T t, int flags) const override {
    auto result = std::make_unique<TrajectoryEvaluation<T>>();
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

  void AddToProblem(ceres::Problem& problem,
                    const time_init_t &times,
                    Meta& meta,
                    std::vector<double*> &parameter_blocks,
                    std::vector<size_t> &parameter_sizes) const override {

  }

};

} // namespace trajectories
} // namespace taser

#endif //TASERV2_UNIFORM_SO3_SPLINE_TRAJECTORY_H
