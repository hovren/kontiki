//
// Created by hannes on 2018-01-10.
//

#include <iostream>
#include <taser/trajectories/simple_multi_trajectory.h>
#include <taser/trajectory_estimator.h>
#include <taser/measurements/position_measurement.h>

#include <Eigen/Dense>

using namespace taser;
using namespace taser::trajectories;
using namespace taser::measurements;

int main() {
  std::cout << "Starting" << std::endl;

  auto traj = std::make_shared<SimpleMultiTrajectory>();
  double wa, wb;
  wa = traj->AWeight();
  wb = traj->BWeight();
  std::cout << "FIRST wa=" << wa << ", wb=" << wb << std::endl;

  traj->foo_a->AddVector(Eigen::Vector3d(1, 2, 3));
  traj->foo_a->AddVector(Eigen::Vector3d(11, 11, 12));
  traj->foo_a->AddVector(Eigen::Vector3d(1000, 1000, 1000));

  traj->foo_b->AddVector(Eigen::Vector3d(100, 100, 100));

  std::cout << "FIRST DONE" << std::endl;
  traj->set_AWeight(2.0);
  traj->set_BWeight(10.5);

  wa = traj->AWeight();
  wb = traj->BWeight();
  std::cout << "SECOND wa=" << wa << ", wb=" << wb << std::endl;

//
//  std::vector<double*> vec = {
//      new double[3]{2., 2., 2.},
//      new double[3]{5, 6, 7}
//  };
//
//  double** data = vec.data();
//  double const* const* data_ptr = (double const* const*) data;
//
//  for (int i=0; i < 2; ++i) {
////    auto ptr = &data[i][0];
//    double const* ptr = &data_ptr[i][0];
//    std::cout << "data[" << i <<"]: " << data[i] << ", " << Eigen::Map<Eigen::Vector3d>(data[i]).transpose() << std::endl;
//    std::cout << "data[" << i <<"]: " << ptr << ", " << Eigen::Map<const Eigen::Vector3d>(ptr).transpose() << std::endl;
//  }
//
//  auto ph = std::make_shared<PointerHolder<double>>(data_ptr);
//  for (int i=0; i < 2; ++i) {
//    auto ptr = ph->Parameter(i);
//    std::cout << "ph[" << i << "]: " << ptr << ", "
//    << Eigen::Map<Eigen::Vector3d>(ptr).transpose() << std::endl;
//  }
//
//  auto s1 = ph->Slice(0, 5);
//  std::cout << "s1[0]: " << s1->Parameter(0) << std::endl;
//  auto s2 = ph->Slice(1, 2);
//  std::cout << "s1[0]: " << s2->Parameter(0) << std::endl;
//
////  return 0;
//
//  taser::trajectories::detail::SimpleMultiMeta sm;
//  sm.a.n = 2;
//  sm.a.foo = 1;
//  sm.b.n = 0;
//  sm.b.foo = 1;
//
//  auto view = SimpleMultiTrajectory::template Map<double>((double const* const*) data, sm);
//  std::cout << "View t=1.3: " << view.Position(1.3) << std::endl;

  std::vector<std::shared_ptr<PositionMeasurement>> meas;

  std::vector<double> times ={0.1, 1.1, 2.2, 3.3}; //  {2.1};

  for (auto t : times) {
    auto p = traj->Position(t);
    meas.push_back(std::make_shared<PositionMeasurement>(t, p));
  }

  for (auto m : meas) {
    auto view = traj->AsView();
    std::cout << "t=" << m->t << " p="<< m->p.transpose() << " Measure=" << m->Measure<double, SimpleMultiTrajectory>(view).transpose() << std::endl;
  }

  auto traj2 = std::make_shared<SimpleMultiTrajectory>();
  for (auto i=0; i < 3; ++i)
    traj2->foo_a->AddVector(Eigen::Vector3d::Zero());
  for (auto i=0; i < 1; ++i)
    traj2->foo_b->AddVector(Eigen::Vector3d::Zero());

  TrajectoryEstimator<SimpleMultiTrajectory> estimator(traj2);

  for (auto m : meas) {
    estimator.AddMeasurement(m);
  }

  auto summary = estimator.Solve();
  std::cout << summary.FullReport() << std::endl;

  std::cout << "Measurements" << std::endl;
  for (auto m : meas) {
    auto view = traj2->AsView();
    std::cout << "t=" << m->t << " p="<< m->p.transpose() << " Measure=" << m->Measure<double, SimpleMultiTrajectory>(view).transpose() << std::endl;
  }

  std::cout << "Vectors A" << std::endl;
  for (auto v : traj2->foo_a->AsView().vectors()) {
    std::cout << v.transpose() << std::endl;
  }

  std::cout << "Vectors B" << std::endl;
  for (auto v : traj2->foo_b->AsView().vectors()) {
    std::cout << v.transpose() << std::endl;
  }

  std::cout << "Weights: wa=" << traj2->AWeight() << ", wb=" << traj2->BWeight() << std::endl;

  std::cout << "DONE" << std::endl;
  return 0;
}