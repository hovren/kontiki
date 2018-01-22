//
// Created by hannes on 2017-11-29.
//

#ifndef TASERV2_LINEAR_TRAJECTORY_H
#define TASERV2_LINEAR_TRAJECTORY_H

#include "trajectory.h"
#include "trajectory_estimator.h"
#include "dataholders/vectorholder.h"

#include <Eigen/Dense>
#include <ceres/ceres.h>

namespace taser {
namespace trajectories {

namespace detail {

struct LinearMeta : public taser::trajectories::MetaBase {
  LinearMeta() {};
  LinearMeta(double t0) : t0(t0) {};

  double t0; // Time origin

  int NumParameters() const override {
    return 0;
  }
};


template<typename T>
class LinearView  : public ViewBase<T, LinearMeta> {
  using Vector3 = Eigen::Matrix<T, 3, 1>;
  using Result = std::unique_ptr<TrajectoryEvaluation<T>>;
 public:
  using Meta = LinearMeta;

  // Inherit ViewBase constructor
  using ViewBase<T, LinearMeta>::ViewBase;

  T t0() const {
    return T(this->meta_.t0);
  }

  const Vector3 constant() const {
    const Vector3 v = Eigen::Map<const Vector3>(this->holder_->Parameter(0));
    return v;
  }

  Result Evaluate(const T t, const int flags) const override {
//    std::cout << "t="<<t<<", t0=" << t0() << ", c=" << constant().transpose() << std::endl;
    auto result = std::make_unique<TrajectoryEvaluation<T>>(flags);
    if (result->needs.Position())
      result->position = constant() * (t - t0());
    if (result->needs.Velocity())
      result->velocity = constant();
    if (result->needs.Acceleration())
      result->acceleration.setZero();
    if (result->needs.Orientation())
      result->orientation = this->calculate_orientation(t);
    if (result->needs.AngularVelocity())
      result->angular_velocity = constant();
    return result;
  }

  double MinTime() const override {
    return -std::numeric_limits<double>::infinity();
  }

  double MaxTime() const override {
    return std::numeric_limits<double>::infinity();
  }

 protected:

  Eigen::Quaternion<T> calculate_orientation(T t) const {
    Vector3 c = constant();
    T theta = c.norm() * (t - t0());
    Vector3 n = c.normalized();
    Eigen::AngleAxis<T> aa(theta, n);
    return Eigen::Quaternion<T>(aa);
  }
}; // ::View


} // namespace detail


// Linear trajectory such that we have
// position p(t) = c * (t - t0)
// angular velocity w(t) = c
// Here c is a 3D vector parameter and t0 is the time origin (metadata)
class LinearTrajectory : public TrajectoryBase<detail::LinearView> {
  using Vector3 = Eigen::Vector3d;
 public:
  static constexpr const char* CLASS_ID = "Linear";

  LinearTrajectory(double t0, const Vector3& k) : TrajectoryBase(new dataholders::VectorHolder<double>()) {
    // One 3D parameter: the constant
    this->holder_->AddParameter(3); // Parameter(0)

    set_t0(t0);
    set_constant(k);
  }

  LinearTrajectory() : LinearTrajectory(0, Eigen::Vector3d(1, 1, 1)) {};


  Vector3 constant() const {
    return AsView().constant();
  }

  void set_constant(const Vector3 &k) {
    Eigen::Map<Vector3> c(holder_->Parameter(0));
    c = k;
  }

  double t0() const {
    return AsView().t0();
  }

  void set_t0(double x) {
    this->meta_.t0 = x;
  }

  // Add to problem, fill Meta struct, return parameter blocks
  void AddToProblem(ceres::Problem& problem,
                      const time_init_t &times,
                      Meta& meta,
                      std::vector<double*> &parameter_blocks,
                      std::vector<size_t> &parameter_sizes) const override {
    // Fill meta
    meta = meta_;

    // Define/add parameter blocks and add to problem
    // In this case we have only one: the constant slope parameter
    auto ptr = holder_->Parameter(0);
    problem.AddParameterBlock(ptr, 3);
    parameter_blocks.push_back(ptr);
    parameter_sizes.push_back(3);
  }
};

} // namespace trajectories
} // namespace taser

#endif //TASERV2_LINEAR_TRAJECTORY_H
