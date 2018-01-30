//
// Created by hannes on 2018-01-30.
//

#include <iostream>

#include <Eigen/Dense>
#include <entity/entity.h>
#include <taser/trajectories/linear_trajectory.h>

#include <taser/trajectory_estimator.h>
#include <taser/measurements/position_measurement.h>
#include <taser/measurements/gyroscope_measurement.h>
#include <taser/sensors/imu.h>

using namespace taser;
using namespace taser::trajectories;
using namespace taser::sensors;

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

template<typename TrajectoryType, typename ImuType>
void gyroscope(const type::Trajectory<TrajectoryType, double> &trajectory, const type::Imu<ImuType, double> &imu) {
  double t = 3.0;
  Eigen::Vector3d w = imu.template Gyroscope<TrajectoryType>(trajectory, t);
  std::cout << "Gyro at t="<<t<< " is " << w.transpose() << std::endl;
};

int main() {
  auto linear = std::make_shared<LinearTrajectory>(1.0, Eigen::Vector3d(1, 2, 4));

  double t = 5.0;
  std::cout << "Pos: " << linear->Position(t).transpose() << std::endl;

  do_things<LinearTrajectory>(*linear);
  do_things2<LinearTrajectory>(*linear);

  TrajectoryEstimator<LinearTrajectory> estimator(linear);

  auto m1 = std::make_shared<measurements::PositionMeasurement>(3, Eigen::Vector3d(5, 1, 3));

  estimator.AddMeasurement(m1);

  std::cout << "BEFORE: constant=" << linear->constant().transpose() << std::endl;
  auto summary = estimator.Solve();
  std::cout << summary.BriefReport() << std::endl;
  std::cout << "AFTER: constant=" << linear->constant().transpose() << std::endl;

  auto basic_imu = std::make_shared<BasicImu>();
  auto constant_imu = std::make_shared<ConstantBiasImu>(Eigen::Vector3d(1, 2, 3), Eigen::Vector3d(1, 1, 1));

  gyroscope<LinearTrajectory, BasicImu>(*linear, *basic_imu);
  gyroscope<LinearTrajectory, ConstantBiasImu>(*linear, *constant_imu);

  auto mg1 = std::make_shared<measurements::GyroscopeMeasurement<BasicImu>>(basic_imu, 4.0, Eigen::Vector3d(2, 2, 2));

  estimator.AddMeasurement(mg1);

  std::cout << "BEFORE: constant=" << linear->constant().transpose() << std::endl;
  summary = estimator.Solve();
  std::cout << summary.BriefReport() << std::endl;
  std::cout << "AFTER: constant=" << linear->constant().transpose() << std::endl;

  auto mg2 = std::make_shared<measurements::GyroscopeMeasurement<ConstantBiasImu>>(constant_imu, 3.0, Eigen::Vector3d(2, 2, 2));
  estimator.AddMeasurement(mg2);

  std::cout << "BEFORE: constant=" << linear->constant().transpose() << std::endl;
  std::cout << "BEFORE: gbias=" << constant_imu->gyro_bias().transpose() << std::endl;
  summary = estimator.Solve();
  std::cout << summary.BriefReport() << std::endl;
  std::cout << "AFTER: constant=" << linear->constant().transpose() << std::endl;
  std::cout << "AFTER: gbias=" << constant_imu->gyro_bias().transpose() << std::endl;

  return 0;
}