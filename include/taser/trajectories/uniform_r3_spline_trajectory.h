#ifndef TASERV2_UNIFORM_R3_SPLINE_TRAJECTORY_H
#define TASERV2_UNIFORM_R3_SPLINE_TRAJECTORY_H

#include <vector>
#include <cmath>

#include "trajectory.h"
#include "../trajectory_estimator.h"
#include "dataholders/vectorholder.h"

namespace taser {
namespace trajectories {

namespace detail {

struct UniformR3SplineMeta : public MetaBase {
  double t0; // First valid time
  double dt; // Knot spacing
  size_t n; // Number of knots

  UniformR3SplineMeta() :
      t0(0.0),
      dt(1.0),
      n(0) { };

  int NumParameters() const override {
    return n;
  }
};

template<typename T>
class UniformR3SplineView : public ViewBase<T, UniformR3SplineMeta> {
  using Result = std::unique_ptr<TrajectoryEvaluation<T>>;
 public:
  using Meta = UniformR3SplineMeta;

  // Import constructor
  using ViewBase<T, UniformR3SplineMeta>::ViewBase;

  Result Evaluate(T t, int flags) const override {
    auto result = std::make_unique<TrajectoryEvaluation<T>>();
    return result;
  }
};

} // namespace detail

class UniformR3SplineTrajectory : public TrajectoryBase<detail::UniformR3SplineView> {
 public:
  static constexpr const char* CLASS_ID = "UniformR3Spline";

  UniformR3SplineTrajectory(double dt, double t0) :
      TrajectoryBase(new dataholders::VectorHolder<double>()) {
    // Create 4 initial control points necessary
    for (int i=0; i < 4; ++i)
      this->holder_->AddParameter(3);

    set_t0(t0);
    set_dt(dt);
  };

  UniformR3SplineTrajectory(double dt) :
    UniformR3SplineTrajectory(dt, 0.0) { };

  UniformR3SplineTrajectory() :
      UniformR3SplineTrajectory(0.0) { };

  void set_t0(double t0) {
    this->meta_.t0 = t0;
  }

  void set_dt(double dt) {
    this->meta_.dt = dt;
  }

  void AddToProblem(ceres::Problem& problem,
                    const time_init_t &times,
                    Meta& meta,
                    std::vector<double*> &parameter_blocks,
                    std::vector<size_t> &parameter_sizes) const {
    // FIXME: Do things
  }
};

} // namespace trajectories
} // namespace taser

#endif //TASERV2_UNIFORM_R3_SPLINE_TRAJECTORY_H
