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

  T t0() const {
    return T(this->meta_.t0);
  }

  T dt() const {
    return T(this->meta_.dt);
  }

  size_t NumKnots() const {
    return this->meta_.n;
  }

  void CalculateIndexAndInterpolationAmount(T t, int& i0, T& u) const {
    T s = (t - t0()) / dt();
    i0 = PotentiallyUnsafeFloor(s);
    u = s - T(i0);
  }

 protected:
  int PotentiallyUnsafeFloor(double x) const {
    return static_cast<int>(std::floor(x));
  }

  // This way of treating Jets are potentially unsafe, hence the function name
  template<typename Scalar, int N>
  int PotentiallyUnsafeFloor(const ceres::Jet<Scalar, N>& x) const {
    return static_cast<int>(std::floor(x.a));
  };
};

template<typename T>
class UniformR3SplineView : public SplineViewBase<T, SplineMeta> {
  using Result = std::unique_ptr<TrajectoryEvaluation<T>>;
  using Vector3 = Eigen::Matrix<T, 3, 1>;
  using Vector3Map = Eigen::Map<Vector3>;
 public:
  using Meta = SplineMeta;

  // Import constructor
  using SplineViewBase<T, SplineMeta>::SplineViewBase;

  const Vector3Map ControlPoint(int i) const {
    return Vector3Map(this->holder_->Parameter(i));
  }

  Vector3Map MutableControlPoint(int i) {
    return Vector3Map(this->holder_->Parameter(i));
  }

  Result Evaluate(T t, int flags) const override {
    auto result = std::make_unique<TrajectoryEvaluation<T>>();

    int i0;
    T u;
    this->CalculateIndexAndInterpolationAmount(t, i0, u);
//    std::cout << "t=" << t << " i0=" << i0 << " u=" << u << std::endl;

    const size_t N = this->NumKnots();
    if ((N < 4) || (i0 < 0) || (i0 > (N - 4))) {
      std::stringstream ss;
      ss << "t=" << t << " i0=" << i0 << " is out of range for spline with ncp=" << N;
      throw std::range_error(ss.str());
    }

    const Eigen::Matrix<T, 1, 4> U(T(1), u, u*u, u*u*u);
    const Eigen::Matrix<T, 4, 1> Q = U * M.cast<T>();

    Eigen::Matrix<T, 3, 1> p = Eigen::Matrix<T, 3, 1>::Zero();

    for (int i=i0; i < i0 + 4; ++i) {
      p += Q(i - i0) * ControlPoint(i);
    }

    result->position = p;

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

  Vector3 ControlPoint(size_t i) {
    return AsView().ControlPoint(i);
  }

  Vector3 operator[](size_t i) {
    return ControlPoint(i);
  }

  void AppendKnot(const Vector3& cp) {
    auto i = this->holder_->AddParameter(3);
    AsView().MutableControlPoint(i) = cp;
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
    if (times.size() != 1) {
      throw std::length_error("Mult i times not implemented yet");
    }

    int i1, i2;
    double u_notused;
    double t1, t2;

    for (auto tt : times) {
      t1 = tt.first;
      t2 = tt.second;
    }

    // Find control point range
    AsView().CalculateIndexAndInterpolationAmount(t1, i1, u_notused);
    AsView().CalculateIndexAndInterpolationAmount(t2, i2, u_notused);
    std::cout << "1: " << t1 << ", " << i1 << " --- 2: " << t2 << ", " << i2 << std::endl;

    for (int i=i1; i < i2 + 4; ++i) {
      auto ptr = this->holder_->Parameter(i);
      const int size = 3;
      parameter_blocks.push_back(ptr);
      parameter_sizes.push_back(size);
      problem.AddParameterBlock(ptr, size);
    }

    // Set meta
    meta.dt = dt();
    meta.n = (i2 + 4 - i1 + 1);
    meta.t0 = t0() + i1 * dt();
  }
};

} // namespace trajectories
} // namespace taser

#endif //TASERV2_UNIFORM_R3_SPLINE_TRAJECTORY_H
