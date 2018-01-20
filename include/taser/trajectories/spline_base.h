//
// Created by hannes on 2018-01-15.
//

#ifndef TASERV2_SPLINE_BASE_H
#define TASERV2_SPLINE_BASE_H

#include <Eigen/Dense>

#include "trajectory.h"
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

struct SplineSegmentMeta : public MetaBase {
  double t0; // First valid time
  double dt; // Knot spacing
  size_t n; // Number of knots

  SplineSegmentMeta(double dt, double t0) :
      dt(dt),
      t0(t0),
      n(0) { };

  SplineSegmentMeta() :
      SplineSegmentMeta(1.0, 0.0) { };

  int NumParameters() const override {
    return n;
  }

  double MinTime() const {
    return t0;
  }

  double MaxTime() const {
    return t0 + (n-3) * dt;
  }
};

// FIXME: Splines consists of segments. We should thus implement segment views and then have a general wrapper to combine them.
struct SplineMeta : public MetaBase {

  std::vector<SplineSegmentMeta> segments;

  int NumParameters() const override {
    /*return std::accumulate(segments.begin(), segments.end(),
                           0, [](int n, SplineSegmentMeta& meta) {
          return n + meta.NumParameters();
        });*/
    int n = 0;
    for (auto &segment_meta : segments) {
      n += segment_meta.NumParameters();
    }
    return n;
  }

  double MinTime() const {
    if (segments.size() == 1)
      return segments[0].MinTime();
    else
      throw std::runtime_error("Concrete splines must have exactly one segment");
  }

  double MaxTime() const {
    if (segments.size() == 1)
      return segments[0].MaxTime();
    else
      throw std::runtime_error("Concrete splines must have exactly one segment");
  }
};


template<typename T>
class SplineSegmentViewBase : public ViewBase<T, SplineSegmentMeta> {
 public:
  // Inherit constructor
  using ViewBase<T, SplineSegmentMeta>::Meta;
  using ViewBase<T, SplineSegmentMeta>::ViewBase;

  T t0() const {
    return T(this->meta_.t0);
  }

  T dt() const {
    return T(this->meta_.dt);
  }

  size_t NumKnots() const {
    return this->meta_.n;
  }

  double MinTime() const override {
    return this->meta_.MinTime();
  }

  double MaxTime() const override {
    return this->meta_.MaxTime();
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


template<template<typename> typename ViewTemplate>
class SplinedTrajectoryBase : public TrajectoryBase<ViewTemplate> {
 public:
  using Meta = typename TrajectoryBase<ViewTemplate>::Meta;

  SplinedTrajectoryBase() :
    TrajectoryBase<ViewTemplate>(new dataholders::VectorHolder<double>()) { };

//  SplinedTrajectoryBase(double dt, double t0) :
//      TrajectoryBase<ViewTemplate>(new dataholders::VectorHolder<double>()) {
//  };
//
//  SplinedTrajectoryBase(double dt) :
//      SplinedTrajectoryBase(dt, 0.0) { };
//
//  SplinedTrajectoryBase() :
//      SplinedTrajectoryBase(1.0) { };

  double t0() const {
    return this->AsView().t0();
  }

  double dt() const {
    return this->AsView().dt();
  }

  size_t NumKnots() const {
    return this->AsView().NumKnots();
  }
};


} // namespace detail
} // namespace trajectories
} // namespace taser

#endif //TASERV2_SPLINE_BASE_H
