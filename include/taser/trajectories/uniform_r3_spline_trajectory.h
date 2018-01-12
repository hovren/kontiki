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

static const Eigen::Matrix4d M = (Eigen::Matrix4d() <<
    1. / 6., 4. / 6.,  1. / 6., 0,
    -3. / 6.,       0,  3. / 6., 0,
    3. / 6., -6. / 6,  3. / 6., 0,
    -1. / 6.,   3./6., -3. / 6., 1./6.).finished();

static const Eigen::Matrix4d M_cumul = (Eigen::Matrix4d() <<
    6. / 6.,  5. / 6.,  1. / 6., 0,
    0. / 6.,  3. / 6.,  3. / 6., 0,
    0. / 6., -3. / 6.,  3. / 6., 0,
    0. / 6.,  1. / 6., -2. / 6., 1. / 6.).finished();

struct SplineMeta : public MetaBase {
  double t0; // First valid time
  double dt; // Knot spacing
  size_t n; // Number of knots

  SplineMeta(double dt, double t0) :
      t0(0.0),
      dt(1.0),
      n(0) { };

  SplineMeta() :
       SplineMeta(1.0, 0.0) { };

  int NumParameters() const override {
    return n;
  }
};

template<typename T, typename Meta>
class SplineViewBase : public ViewBase<T, Meta> {
 public:
  // Inherit constructor
  using ViewBase<T, Meta>::ViewBase;

  double t0() const {
    return this->meta_.t0;
  }

  double dt() const {
    return this->meta_.dt;
  }

  size_t NumKnots() const {
    return this->meta_.n;
  }
};

template<typename T>
class UniformR3SplineView : public SplineViewBase<T, SplineMeta> {
  using Result = std::unique_ptr<TrajectoryEvaluation<T>>;
 public:
  using Meta = SplineMeta;

  // Import constructor
  using SplineViewBase<T, SplineMeta>::SplineViewBase;

  Result Evaluate(T t, int flags) const override {
    auto result = std::make_unique<TrajectoryEvaluation<T>>();
    return result;
  }
};

} // namespace detail

class UniformR3SplineTrajectory : public TrajectoryBase<detail::UniformR3SplineView> {
  using Vector3 = Eigen::Vector3d;
 public:
  static constexpr const char* CLASS_ID = "UniformR3Spline";

  UniformR3SplineTrajectory(double dt, double t0) :
      TrajectoryBase(new dataholders::VectorHolder<double>(), Meta(dt, t0)) {
  };

  UniformR3SplineTrajectory(double dt) :
    UniformR3SplineTrajectory(dt, 0.0) { };

  UniformR3SplineTrajectory() :
      UniformR3SplineTrajectory(1.0) { };

  double t0() const {
    return AsView().t0();
  }

  double dt() const {
    return AsView().dt();
  }

  void AppendKnot(const Vector3& cp) {
    auto i = this->holder_->AddParameter(3);
    Eigen::Map<Vector3> cpmap(holder_->Parameter(i));
    cpmap = cp;
    this->meta_.n += 1;
  }

  size_t NumKnots() const {
    return AsView().NumKnots();
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
