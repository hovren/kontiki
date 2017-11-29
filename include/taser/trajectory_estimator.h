//
// Created by hannes on 2017-11-29.
//

#ifndef TASERV2_TRAJECTORY_ESTIMATOR_H
#define TASERV2_TRAJECTORY_ESTIMATOR_H

namespace taser {
template <template<typename> typename TrajectoryModel>
class TrajectoryEstimator {
  using TrajectoryImpl = TrajectoryModel<double>;

 public:
  TrajectoryEstimator(std::shared_ptr<TrajectoryImpl> trajectory) : trajectory_(trajectory) {};

  auto trajectory() const {
    return trajectory_;
  }
  
 protected:
  std::shared_ptr<TrajectoryImpl> trajectory_;
};

} // namespace taser
#endif //TASERV2_TRAJECTORY_ESTIMATOR_H
