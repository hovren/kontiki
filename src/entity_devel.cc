//
// Created by hannes on 2018-01-30.
//

#include <iostream>

#include <Eigen/Dense>
#include <entity/entity.h>
#include <taser/trajectories/linear_trajectory.h>
#include <taser/trajectories/uniform_r3_spline_trajectory.h>

#include <taser/trajectory_estimator.h>
#include <taser/measurements/position_measurement.h>
#include <taser/measurements/static_rscamera_measurement.h>
#include <taser/cameras/pinhole.h>
#include <taser/cameras/atan.h>
#include <taser/sfm/view.h>
#include <taser/sfm/landmark.h>
#include <taser/sfm/observation.h>
#include <entity/paramstore/empty_pstore.h>

using namespace taser;
using namespace taser::trajectories;
using namespace taser::cameras;
using namespace taser::measurements;

template<typename TrajectoryModel>
void do_trajectory(const type::Trajectory<TrajectoryModel, double> &trajectory) {
  std::cout << "from func, pos=" << trajectory.Position(0.3).transpose() << std::endl;
}

int main() {
  auto r3_spline = std::make_shared<UniformR3SplineTrajectory>(1.0, 0.0);
  std::cout << "BEFORE: nknots=" << r3_spline->NumKnots() << std::endl;
  for (auto& cp : {
      Eigen::Vector3d(1, 1, 1),
      Eigen::Vector3d(1, 2, 3),
      Eigen::Vector3d(-1, 2, 5),
      Eigen::Vector3d(-1, 7, -2),
      Eigen::Vector3d(1, 1, 1),
      Eigen::Vector3d(1, 2, 3),
      Eigen::Vector3d(-1, 2, 5),
      Eigen::Vector3d(-1, 7, -2),
      Eigen::Vector3d(1, 1, 1),
      Eigen::Vector3d(1, 2, 3),
      Eigen::Vector3d(-1, 2, 5),
      Eigen::Vector3d(-1, 7, -2)
  }) {
    r3_spline->AppendKnot(cp);
  }

  std::cout << "AFTER: nknots=" << r3_spline->NumKnots() << std::endl;
  std::cout << "Valid time: " << r3_spline->MinTime() << " to " << r3_spline->MaxTime() << std::endl;
  double t = 0.3;
  std::cout << "pos: " << r3_spline->Position(t).transpose() << std::endl;

  do_trajectory<UniformR3SplineTrajectory>(*r3_spline);

  TrajectoryEstimator<UniformR3SplineTrajectory> estimator(r3_spline);

  auto camera = std::make_shared<PinholeCamera>(1920, 1080, 0.2);
  auto v1 = std::make_shared<View>(0, 0.);
  auto v2 = std::make_shared<View>(7, 2.0);
  auto lm = std::make_shared<Landmark>();
  auto ref = v1->create_observation(lm, 400, 400);
  auto obs = v2->create_observation(lm, 300, 200);
  lm->set_reference(ref);

  using MClass = StaticRsCameraMeasurement<PinholeCamera>;
  auto m = std::make_shared<MClass>(camera, obs);

  estimator.AddMeasurement(m);

//  auto m1 = std::make_shared<PositionMeasurement>(0.2, Eigen::Vector3d(1, 2, 3));
//  auto m2 = std::make_shared<PositionMeasurement>(0.45, Eigen::Vector3d(-1, 1, 0));
//  estimator.AddMeasurement<PositionMeasurement>(m1);
//  estimator.AddMeasurement<PositionMeasurement>(m2);

  auto summary = estimator.Solve();

  std::cout << summary.FullReport() << std::endl;

  return 0;
}