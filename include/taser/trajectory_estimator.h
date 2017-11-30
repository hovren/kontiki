//
// Created by hannes on 2017-11-29.
//

#ifndef TASERV2_TRAJECTORY_ESTIMATOR_H
#define TASERV2_TRAJECTORY_ESTIMATOR_H

#include <ceres/ceres.h>

#include <iostream>

namespace taser {
template <template<typename> typename TrajectoryModel>
class TrajectoryEstimator {
  using TrajectoryImpl = TrajectoryModel<double>;

 public:
  TrajectoryEstimator(std::shared_ptr<TrajectoryImpl> trajectory) : trajectory_(trajectory) {};

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

  template<typename T>
  T test(T x) {
    return x*2;
  }

  ceres::Problem& problem() {
    return problem_;
  }

 protected:
  std::shared_ptr<TrajectoryImpl> trajectory_;

  ceres::Problem problem_;
};

} // namespace taser
#endif //TASERV2_TRAJECTORY_ESTIMATOR_H
