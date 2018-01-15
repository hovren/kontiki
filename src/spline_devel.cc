#include <iostream>
#include <vector>

#include <taser/trajectories/uniform_r3_spline_trajectory.h>
#include <taser/trajectory_estimator.h>
#include <taser/measurements/position_measurement.h>

using namespace taser;
using namespace taser::trajectories;
using namespace taser::measurements;

int main() {
  std::cout << "Begin" << std::endl;

  auto traj = std::make_shared<UniformR3SplineTrajectory>();

  traj->AppendKnot(Eigen::Vector3d(1, 1, 1));
  traj->AppendKnot(Eigen::Vector3d(1, 5, -1));
  traj->AppendKnot(Eigen::Vector3d(2, 3, -1));
  traj->AppendKnot(Eigen::Vector3d(1, 5, -4));
  traj->AppendKnot(Eigen::Vector3d(-2, 12, -7));
  traj->AppendKnot(Eigen::Vector3d(-3, 10, -8));

  std::cout << "dt=" << traj->dt() << ", t0=" << traj->t0();
  std::cout << " nknots=" << traj->NumKnots() << std::endl;

  std::cout << "::KNOTS" << std::endl;
  for (int i=0; i < traj->NumKnots(); ++i) {
    std::cout << i << ": " << traj->ControlPoint(i).transpose() << std::endl;
  }

  auto m1 = std::make_shared<PositionMeasurement>(0.1, Eigen::Vector3d(1, 1, 2));
  auto m2 = std::make_shared<PositionMeasurement>(1.2, Eigen::Vector3d(3, -1, -2));
  auto m3 = std::make_shared<PositionMeasurement>(1.8, Eigen::Vector3d(3.2, -1.4, -2.2));
  auto m4 = std::make_shared<PositionMeasurement>(2.2, Eigen::Vector3d(2.6, -1.1, -1.0));

  std::vector<std::shared_ptr<PositionMeasurement>> meas = {m1, m2, m3, m4};

  TrajectoryEstimator<UniformR3SplineTrajectory> estimator(traj);
  for (auto m : meas){
    estimator.AddMeasurement(m);
  }

  auto summary = estimator.Solve();

  std::cout << summary.FullReport() << std::endl;

  for (auto m : meas){
    Eigen::Vector3d phat = m->Measure<double, UniformR3SplineTrajectory>(traj->AsView());
    std::cout << "t=" << m->t << " p=" << m->p.transpose() << "  -->  " << phat.transpose() << std::endl;
  }

  std::cout << "::KNOTS" << std::endl;
  for (int i=0; i < traj->NumKnots(); ++i) {
    std::cout << i << ": " << traj->ControlPoint(i).transpose() << std::endl;
  }

  return 0;
}