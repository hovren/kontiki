#include <Eigen/Dense>
#include <taser/sensors/imu.h>
#include <taser/trajectories/linear_trajectory.h>
#include <taser/measurements/gyroscope_measurement.h>

using namespace taser::sensors;
using namespace taser::trajectories;
using namespace taser::measurements;

int main() {

  auto trajectory = std::make_shared<LinearTrajectory>();
  auto imu = std::make_shared<BasicImu>();
  auto m1 = std::make_shared<GyroscopeMeasurement<BasicImu>>(imu, 2.0, Eigen::Vector3d(1, 2, 3));

  std::vector<std::shared_ptr<GyroscopeMeasurement<BasicImu>>> meas = {m1};

  double t = 10.;
  std::cout << "Position: " << trajectory->Position(t).transpose() << std::endl;
  std::cout << "Gyroscope: " << imu->Gyroscope(trajectory, t).transpose() << std::endl;

  for (auto m : meas) {
    std::cout << "t=" << m->t << " w=" << m->w.transpose() << " -> "
              << m->Measure<LinearTrajectory, double>(imu->AsView<LinearTrajectory>(), trajectory->AsView()).transpose()
              << " err=" << m->Error<LinearTrajectory, double>(imu->AsView<LinearTrajectory>(), trajectory->AsView()).transpose() << std::endl;
  }

  std::cout << "DONE" << std::endl;
  return 0;
}