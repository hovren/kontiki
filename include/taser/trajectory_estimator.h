//
// Created by hannes on 2017-11-29.
//

#ifndef TASERV2_TRAJECTORY_ESTIMATOR_H
#define TASERV2_TRAJECTORY_ESTIMATOR_H

#include <iostream>
#include <vector>
#include <memory>

#include <ceres/ceres.h>

#include "trajectories/trajectory.h"

namespace taser {

template <typename TrajectoryModel>
class TrajectoryEstimator {
  using Meta = typename TrajectoryModel::Meta;
  static ceres::Problem::Options DefaultProblemOptions() {
    ceres::Problem::Options options;
    options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    return options;
  }
 public:
  TrajectoryEstimator(std::shared_ptr<TrajectoryModel> trajectory) :
      trajectory_(trajectory),
      problem_(DefaultProblemOptions()) { };

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

  bool AddTrajectoryForTimes(const time_init_t &times,
                             Meta &meta,
                             std::vector<entity::ParameterInfo<double>> &parameter_info) {
    CheckTimeSpans(times);
    trajectory_->AddToProblem(problem_, times, meta, parameter_info);
    return true;
  }

 protected:

  // Check the following
  // 1) All time spans are valid for the current trajectory
  // 2) Time spans are ordered (ascending)
  // Throw std::range_error on failure
  void CheckTimeSpans(const time_init_t &times) {
    int i=0;
    double t1_prev;

    for (auto &tspan : times) {
      double t1, t2;
      std::tie(t1, t2) = tspan;

      // Valid for trajectory?
      if ((t1 < trajectory_->MinTime()) ||
          (t2 >= trajectory_->MaxTime())) {
        throw std::range_error("Time span out of range for trajectory");
      }

      // Ordered?
      if (t1 > t2) {
        throw std::range_error("At least one time span begins before it ends");
      }
      else if((i > 0) && (t1 < t1_prev)) {
        throw std::range_error("Time spans are not ordered");
      }

      t1_prev = t1;
      i += 1;
    }
  }

  std::shared_ptr<TrajectoryModel> trajectory_;
  ceres::Problem problem_;
};

} // namespace taser
#endif //TASERV2_TRAJECTORY_ESTIMATOR_H
