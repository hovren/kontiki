#include <iostream>
#include <vector>

#include <taser/trajectories/uniform_r3_spline_trajectory.h>
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

  auto traj = std::make_shared<UniformR3SplineTrajectory>(0.34, 1.22);

  traj->AppendKnot(Eigen::Vector3d(1, 1, 1));
  traj->AppendKnot(Eigen::Vector3d(1, 5, -1));
  traj->AppendKnot(Eigen::Vector3d(2, 3, -1));
  traj->AppendKnot(Eigen::Vector3d(1, 5, -4));
  traj->AppendKnot(Eigen::Vector3d(-2, 12, -7));
  traj->AppendKnot(Eigen::Vector3d(-3, 10, -8));
  traj->AppendKnot(Eigen::Vector3d(-4, 11, -8));
  traj->AppendKnot(Eigen::Vector3d(-3, 2, -12));
  traj->AppendKnot(Eigen::Vector3d(-6, 5, -1));

  std::cout << "dt=" << traj->dt() << ", t0=" << traj->t0();
  std::cout << " nknots=" << traj->NumKnots() << std::endl;
  std::cout << "valid time: " << traj->MinTime() << " - " << traj->MaxTime() << std::endl;

  std::cout << "::KNOTS" << std::endl;
  for (int i=0; i < traj->NumKnots(); ++i) {
    std::cout << i << ": " << traj->ControlPoint(i).transpose() << std::endl;
  }

  TrajectoryEstimator<UniformR3SplineTrajectory> estimator(traj);

#if 0
  auto m1 = std::make_shared<PositionMeasurement>(0.1, Eigen::Vector3d(1, 1, 2));
  auto m2 = std::make_shared<PositionMeasurement>(1.2, Eigen::Vector3d(3, -1, -2));
  auto m3 = std::make_shared<PositionMeasurement>(1.8, Eigen::Vector3d(3.2, -1.4, -2.2));
  auto m4 = std::make_shared<PositionMeasurement>(2.2, Eigen::Vector3d(2.6, -1.1, -1.0));

  std::vector<std::shared_ptr<PositionMeasurement>> meas = {m1, m2, m3, m4};
#else
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

#endif
  for (auto m : meas){
    estimator.AddMeasurement(m);
  }

  auto summary = estimator.Solve();

  std::cout << summary.FullReport() << std::endl;

#if 0
  for (auto m : meas){
    Eigen::Vector3d phat = m->Measure<double, UniformR3SplineTrajectory>(traj->AsView());
    std::cout << "t=" << m->t << " p=" << m->p.transpose() << "  -->  " << phat.transpose() << std::endl;
  }
#endif

  std::cout << "::KNOTS" << std::endl;
  for (int i=0; i < traj->NumKnots(); ++i) {
    std::cout << i << ": " << traj->ControlPoint(i).transpose() << std::endl;
  }

  return 0;
}