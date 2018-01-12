//
// Created by hannes on 2017-11-29.
//

#ifndef TASERV2_TRAJECTORY_ESTIMATOR_H
#define TASERV2_TRAJECTORY_ESTIMATOR_H

#include <iostream>
#include <vector>

#include <ceres/ceres.h>

#include "trajectories/trajectory.h"

namespace taser {

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

    // FIXME: Don't hardcode thread counts
    options.num_linear_solver_threads = 4;
    options.num_threads = 3;

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

  bool AddTrajectoryForTimes(const trajectories::time_init_t &times,
                             Meta &meta,
                             std::vector<double *> &parameter_blocks,
                             std::vector<size_t> &parameter_sizes) {
    trajectory_->AddToProblem(problem_, times, meta, parameter_blocks, parameter_sizes);
    return true;
  }

 protected:
  std::shared_ptr<TrajectoryModel> trajectory_;
  ceres::Problem problem_;
};

} // namespace taser
#endif //TASERV2_TRAJECTORY_ESTIMATOR_H
