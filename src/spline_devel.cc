#include <iostream>
#include <vector>

#include <sophus/se3.hpp>

#include <taser/trajectories/uniform_se3_spline_trajectory.h>
#include <taser/trajectory_estimator.h>
#include <taser/measurements/position_measurement.h>
#include <taser/cameras/pinhole.h>
#include <taser/measurements/static_rscamera_measurement.h>
#include <taser/sfm/view.h>

using namespace taser;
using namespace taser::trajectories;
using namespace taser::measurements;

int main() {
  std::cout << "Begin" << std::endl;

  using SE3Type = Sophus::SE3d;
  using Point = SE3Type::Point;
  using SO3Type = Sophus::SO3d;

  Eigen::Matrix4d Z;
  Z << 1, 0, 0, 10, 0, 1, 0, 20, 0, 0, 1, 30, 0, 0, 0, 1;
  std::cout<< Z << std::endl;

  Sophus::SE3d T(Z);
  std::cout << T.matrix() << std::endl;

  auto traj = std::make_shared<UniformSE3SplineTrajectory>(0.34, 1.22);

  std::cout << "dt=" << traj->dt() << ", t0=" << traj->t0();
  std::cout << " nknots=" << traj->NumKnots() << std::endl;

  for (int i=0; i < 10; ++i) {
    SE3Type cp = SE3Type(SO3Type::exp(Point(0.2, 0.5, 0.0)), Point(0, 0, 0));
    traj->AppendKnot(cp);
  }

  std::cout << " nknots=" << traj->NumKnots() << std::endl;
  std::cout << "valid time: " << traj->MinTime() << " - " << traj->MaxTime() << std::endl;

  double t = 2.1;
  Eigen::Vector3d p = traj->Position(t);
  std::cout << "Pos: " << p.transpose() << std::endl;

#if 0

  TrajectoryEstimator<UniformSE3SplineTrajectory> estimator(traj);


  auto v1 = std::make_shared<taser::View>(0, 1.22);
  auto v2 = std::make_shared<taser::View>(3, 3.1);
  auto v3 = std::make_shared<taser::View>(2, 1.93);
  auto lm = std::make_shared<taser::Landmark>();
  auto ref = v1->create_observation(lm, 100, 100);
  auto obs = v2->create_observation(lm, 200, 200);
  auto obs2 = v3->create_observation(lm, 103, 110);
  lm->set_reference(ref);

  auto camera = std::make_shared<taser::cameras::PinholeCamera>(640, 480, 0.1);

  using MClass = taser::measurements::StaticRsCameraMeasurement<taser::cameras::PinholeCamera>;

  auto m1 = std::make_shared<MClass>(camera, obs);
  auto m2 = std::make_shared<MClass>(camera, obs2);

  std::vector<std::shared_ptr<MClass>> meas = { m1, m2 };

  for (auto m : meas) {
    std::cout << "Project: " << m->Project<double, UniformR3SplineTrajectory>(traj->AsView()) << std::endl;
  }

  for (auto m : meas){
    estimator.AddMeasurement(m);
  }

  auto summary = estimator.Solve();

  std::cout << summary.FullReport() << std::endl;

  std::cout << "::KNOTS" << std::endl;
  for (int i=0; i < traj->NumKnots(); ++i) {
    std::cout << i << ": " << traj->ControlPoint(i).transpose() << std::endl;
  }

#endif
  return 0;
}