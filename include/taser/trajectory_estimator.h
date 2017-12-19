//
// Created by hannes on 2017-11-29.
//

#ifndef TASERV2_TRAJECTORY_ESTIMATOR_H
#define TASERV2_TRAJECTORY_ESTIMATOR_H

#include <ceres/ceres.h>

#include <iostream>

namespace taser {

using time_span_t = std::pair<double, double>;
using time_init_t = std::initializer_list<time_span_t>;

struct EstimatorOptions {
  // Time spans for which the trajectory should be kept constant
  std::vector<time_span_t> constant_trajectory_spans;
};

template <typename TrajectoryModel>
class TrajectoryEstimator {
  using Meta = typename TrajectoryModel::Meta;

 public:
  TrajectoryEstimator(std::shared_ptr<TrajectoryModel> trajectory) : trajectory_(trajectory) {};

  auto trajectory() const {
    return trajectory_;
  }

  ceres::Solver::Summary Solve() {
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR; // FIXME: SCHUR?
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem_, &summary);
    return summary;
  }

  template<typename MeasurementType>
  void AddMeasurement(std::shared_ptr<MeasurementType> m) {
    m->AddToEstimator(*this);
  }

  ceres::Problem& problem() {
    return problem_;
  }

  bool AddTrajectoryForTimes(const time_init_t &times,
                             Meta &meta,
                             std::vector<double *> &parameter_blocks,
                             std::vector<size_t> &parameter_sizes) {
    trajectory_->AddToEstimator(*this, times, meta, parameter_blocks, parameter_sizes);
  }

 protected:
  std::shared_ptr<TrajectoryModel> trajectory_;
  ceres::Problem problem_;
};

} // namespace taser
#endif //TASERV2_TRAJECTORY_ESTIMATOR_H
