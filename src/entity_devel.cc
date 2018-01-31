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

template<int N>
class FooFactory {
 public:
  template<typename T, typename MetaType>
  class View : public entity::EntityView<T, MetaType> {
   public:
    using entity::EntityView<T, MetaType>::EntityView;

    void foo() {
      std::cout << "My N is " << N << std::endl;
    }
  };
};

class EmptyMeta : public entity::MetaData {
 public:
  size_t NumParameters() const override {
    return 0;
  }
};

template<int N>
class Foo : public entity::Entity<FooFactory<N>::template View, EmptyMeta, entity::EmptyParameterStore<double>> {
  using entity::Entity<FooFactory<N>::template View, EmptyMeta, entity::EmptyParameterStore<double>>::Entity;
};

int main() {
  auto r3_spline = std::make_shared<UniformR3SplineTrajectory>(0.5, 0.0);
  std::cout << "BEFORE: nknots=" << r3_spline->NumKnots() << std::endl;
  for (auto& cp : {
      Eigen::Vector3d(1, 1, 1),
      Eigen::Vector3d(1, 2, 3),
      Eigen::Vector3d(-1, 2, 5),
      Eigen::Vector3d(-1, 7, -2)
  }) {
    r3_spline->AppendKnot(cp);
  }

  std::cout << "AFTER: nknots=" << r3_spline->NumKnots() << std::endl;
  std::cout << "Valid time: " << r3_spline->MinTime() << " to " << r3_spline->MaxTime() << std::endl;
  double t = 0.2;
  std::cout << "pos: " << r3_spline->Position(t).transpose() << std::endl;

  return 0;
}