//
// Created by hannes on 2018-01-30.
//

#include <iostream>

#include <Eigen/Dense>
#include <entity/entity.h>
#include <taser/trajectories/linear_trajectory.h>

#include <taser/trajectory_estimator.h>
#include <taser/measurements/position_measurement.h>
#include <taser/cameras/pinhole.h>
#include <taser/cameras/atan.h>

using namespace taser;
using namespace taser::trajectories;
using namespace taser::cameras;

template<typename CameraType>
void do_camera(const type::Camera<CameraType, double> &camera) {
  Eigen::Vector3d X(1, 2, 30);
  Eigen::Vector2d y = camera.Project(X);
  std::cout << "Project " << X.transpose() << " -> " << y.transpose() << std::endl;
}

int main() {
  auto linear = std::make_shared<LinearTrajectory>(1.0, Eigen::Vector3d(1, 2, 4));

  double t = 5.0;
  std::cout << "Pos: " << linear->Position(t).transpose() << std::endl;

  TrajectoryEstimator<LinearTrajectory> estimator(linear);

  auto m1 = std::make_shared<measurements::PositionMeasurement>(3, Eigen::Vector3d(5, 1, 3));

  estimator.AddMeasurement(m1);

  auto pinhole = std::make_shared<PinholeCamera>(1920, 1080, 0.02);
  do_camera<PinholeCamera>(*pinhole);

  auto atan = std::make_shared<AtanCamera>(1920, 1080, 0.02, 0.8, Eigen::Vector2d(0.5, 0.5));
  do_camera<AtanCamera>(*atan);

  return 0;
}