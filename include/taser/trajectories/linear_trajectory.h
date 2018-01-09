//
// Created by hannes on 2017-11-29.
//

#ifndef TASERV2_LINEAR_TRAJECTORY_H
#define TASERV2_LINEAR_TRAJECTORY_H

#include "trajectory.h"
#include "trajectory_estimator.h"

#include <Eigen/Dense>
#include <ceres/ceres.h>

struct _LinearMeta : public taser::trajectories::MetaBase {
  _LinearMeta() {};
  _LinearMeta(double t0) : t0(t0) {};

  double t0;

  int num_parameters() const override {
    return 0;
  }
};



namespace taser {
namespace trajectories {

// Move outside of namespace to avoid direct instantiation?
template<typename T>
class _LinearView  : public ViewBase<T, _LinearView<T>, _LinearMeta> {
  using Vector3 = Eigen::Matrix<T, 3, 1>;
  using Result = std::unique_ptr<TrajectoryEvaluation<T>>;
 public:
  using Meta = _LinearMeta;

  // Inherit ViewBase constructor
  using ViewBase<T, _LinearView<T>, _LinearMeta>::ViewBase;

  T t0() const {
    return T(this->meta_.t0);
  }

  const Vector3 constant() const {
    const Vector3 v = Eigen::Map<const Vector3>(this->holder_->Parameter(0));
    return v;
  }

  std::unique_ptr<TrajectoryEvaluation<T>> Evaluate(const T t, const int flags) const {
//    std::cout << "t="<<t<<", t0=" << t0() << ", c=" << constant().transpose() << std::endl;
    auto result = std::make_unique<TrajectoryEvaluation<T>>();
    if (!flags)
      throw std::logic_error("Evaluate() called with flags=0");
    if (flags & EvalPosition)
      result->position = constant() * (t - t0());
    if (flags & EvalVelocity)
      result->velocity = constant();
    if (flags & EvalAcceleration)
      result->acceleration.setZero();
    if (flags & EvalOrientation)
      result->orientation = this->calculate_orientation(t);
    if (flags & EvalAngularVelocity)
      result->angular_velocity = constant();
    return result;
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

class LinearTrajectory : public TrajectoryBase<_LinearView> {
  using Vector3 = Eigen::Vector3d;
 public:
  static constexpr const char* CLASS_ID = "Linear";

  LinearTrajectory(double t0, const Vector3& k) : TrajectoryBase(new VectorHolder<double>()) { //: holder_(new VectorHolder<double>()) {
    set_t0(t0);
//    holder_ = new VectorHolder<double>();
    //data_.push_back(new double[3]);
    holder_->AddParameter(3);
    set_constant(k);
  }

  LinearTrajectory() : LinearTrajectory(0, Eigen::Vector3d(1, 1, 1)) {};


  Vector3 constant() const {
    std::cout << "BEGIN LinearTrajectory::constant()" << std::endl;
    auto ptr = holder_->Parameter(0);
    std::cout << "data: " <<  ptr << std::endl;
    Vector3 c = Eigen::Map<Vector3>(ptr);
    std::cout << "END LinearTrajectory::constant()" << std::endl;
    return c;
  }

  void set_constant(const Vector3 &k) {
    Eigen::Map<Vector3> c(holder_->Parameter(0));
    c = k;
  }

  double t0() const {
    return this->meta_.t0;
  }

  void set_t0(double x) {
    this->meta_.t0 = x;
  }



//  std::unique_ptr<TrajectoryEvaluation<double>> Evaluate(const double t, const int flags) const {
//    return View<double>(this).Evaluate(t, flags);
//  }


  // Add to problem, fill Meta struct, return parameter blocks
  void AddToEstimator(TrajectoryEstimator<LinearTrajectory> &estimator,
                      const time_init_t &times,
                      Meta& meta,
                      std::vector<double*> &parameter_blocks,
                      std::vector<size_t> &parameter_sizes) {
    // Fill meta
    meta = meta_; // FIXME: Can we use that meta_ is now part of TrajectoryBase?

    // Define/add parameter blocks and add to problem
    // In this case we have only one: the constant slope parameter
    auto ptr = holder_->Parameter(0);
    estimator.problem().AddParameterBlock(ptr, 3);
    parameter_blocks.push_back(ptr);
    parameter_sizes.push_back(3);
  }
};

} // namespace trajectories
} // namespace taser

#endif //TASERV2_LINEAR_TRAJECTORY_H
