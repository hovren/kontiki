#include <Eigen/Dense>
#include <kontiki/sensors/imu.h>
#include <kontiki/trajectories/linear_trajectory.h>
#include <kontiki/measurements/gyroscope_measurement.h>
#include <kontiki/trajectory_estimator.h>

using namespace kontiki;
using namespace kontiki::sensors;
using namespace kontiki::trajectories;
using namespace kontiki::measurements;

int main() {

  using ImuModel = ConstantBiasImu;
//  using ImuModel = BasicImu;

  auto trajectory = std::make_shared<LinearTrajectory>(0.0, Eigen::Vector3d(2, 4, 6));
  auto imu = std::make_shared<ImuModel>();

  std::cout << "Constant: " << trajectory->constant().transpose() << std::endl;

  std::cout << "BEFORE: " << imu->GyroscopeBias().transpose() << std::endl;
  imu->set_GyroscopeBias(Eigen::Vector3d(6, 6, 6));
  std::cout << "AFTER: " << imu->GyroscopeBias().transpose() << std::endl;

  auto m1 = std::make_shared<GyroscopeMeasurement<ImuModel>>(imu, 2.0, Eigen::Vector3d(1, 2, 3));

  std::vector<std::shared_ptr<GyroscopeMeasurement<ImuModel>>> meas = {m1};

  double t = 10.;
  std::cout << "Position: " << trajectory->Position(t).transpose() << std::endl;
  std::cout << "Gyroscope: " << imu->Gyroscope<LinearTrajectory>(trajectory, t).transpose() << std::endl;

  for (auto m : meas) {
    std::cout << "t=" << m->t << " w=" << m->w.transpose() << " -> "
              << m->Measure<LinearTrajectory, double>(imu->AsView(), trajectory->AsView()).transpose()
              << " err=" << m->Error<LinearTrajectory, double>(imu->AsView(), trajectory->AsView()).transpose() << std::endl;
  }

  TrajectoryEstimator<LinearTrajectory> estimator(trajectory);
  estimator.AddMeasurement(m1);

  auto summary = estimator.Solve();
  std::cout << summary.FullReport() << std::endl;

  for (auto m : meas) {
    std::cout << "t=" << m->t << " w=" << m->w.transpose() << " -> "
              << m->Measure<LinearTrajectory, double>(imu->AsView(), trajectory->AsView()).transpose()
              << " err=" << m->Error<LinearTrajectory, double>(imu->AsView(), trajectory->AsView()).transpose() << std::endl;
  }

  std::cout << "Bias: " << imu->GyroscopeBias().transpose() << std::endl;
  std::cout << "constant: " << trajectory->constant().transpose() << std::endl;

  std::cout << "DONE" << std::endl;
  return 0;
}