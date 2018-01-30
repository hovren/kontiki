//
// Created by hannes on 2018-01-30.
//

#include <iostream>

#include <Eigen/Dense>
#include <entity/entity.h>
#include <taser/trajectories/linear_trajectory.h>

#include <taser/trajectory_estimator.h>
#include <taser/measurements/position_measurement.h>
#include <taser/measurements/static_rscamera_measurement.h>
#include <taser/cameras/pinhole.h>
#include <taser/cameras/atan.h>
#include <taser/sfm/view.h>
#include <taser/sfm/landmark.h>
#include <taser/sfm/observation.h>

using namespace taser;
using namespace taser::trajectories;
using namespace taser::cameras;
using namespace taser::measurements;

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

  std::cout << "Pinhole K=\n" << pinhole->camera_matrix() << std::endl;

  auto atan = std::make_shared<AtanCamera>(1920, 1080, 0.02, 0.8, Eigen::Vector2d(0.5, 0.5));
  do_camera<AtanCamera>(*atan);

  auto v1 = std::make_shared<View>(0, 0.);
  auto v2 = std::make_shared<View>(1, 0.2);
  auto lm = std::make_shared<Landmark>();
  auto ref = v1->create_observation(lm, 200, 200);
  auto obs = v2->create_observation(lm, 300, 300);
  lm->set_reference(ref);

  using SRsPinholeClass = StaticRsCameraMeasurement<PinholeCamera>;
  auto m2 = std::make_shared<SRsPinholeClass>(pinhole, obs);

  using SRsAtanClass = StaticRsCameraMeasurement<AtanCamera>;
  auto m3 = std::make_shared<SRsAtanClass>(atan, obs);

//  estimator.AddMeasurement(m2);
  estimator.AddMeasurement(m3);

  auto summary = estimator.Solve();

  std::cout << summary.FullReport() << std::endl;

  return 0;
}