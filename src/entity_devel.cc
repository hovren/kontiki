//
// Created by hannes on 2018-01-30.
//

#include <iostream>

#include <Eigen/Dense>
#include <entity/entity.h>
#include <taser/trajectories/linear_trajectory.h>

using namespace taser;
using namespace taser::trajectories;

template<typename TrajectoryType>
void do_things(const entity::type::base::ForView<TrajectoryType, TrajectoryView, double> &trajectory) {
  double t = 4.0;
  std::cout << "func pos: " << trajectory.Position(t).transpose() << std::endl;
}

template<typename TrajectoryType>
void do_things2(const type::Trajectory<TrajectoryType, double> &trajectory) {
  double t = 6;
  std::cout << "func2 pos: " << trajectory.Position(t).transpose() << std::endl;
}

int main() {
  auto linear = std::make_shared<LinearTrajectory>(1.0, Eigen::Vector3d(1, 2, 4));

  double t = 5.0;
  std::cout << "Pos: " << linear->Position(t).transpose() << std::endl;

  do_things<LinearTrajectory>(*linear);
  do_things2<LinearTrajectory>(*linear);

  return 0;
}