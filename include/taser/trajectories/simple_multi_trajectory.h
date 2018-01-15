//
// Created by hannes on 2018-01-09.
//

#ifndef TASERV2_SIMPLE_MULTI_TRAJECTORY_H
#define TASERV2_SIMPLE_MULTI_TRAJECTORY_H

#include <vector>
#include <cmath>

#include "trajectory.h"
#include "../trajectory_estimator.h"
#include "dataholders/multiholder.h"
#include "dataholders/vectorholder.h"

namespace taser {
namespace trajectories {

namespace detail {

struct FooMeta : MetaBase {
  int n;
  double foo;

  FooMeta() : n(0), foo(1.0) {  };

  int NumParameters() const override {
    return n;
  }
};

template <typename T>
class FooView : public ViewBase<T, FooMeta> {
  using Result = std::unique_ptr<TrajectoryEvaluation<T>>;
  using Vector3 = Eigen::Matrix<T, 3, 1>;
 public:
  using Meta = FooMeta;
  using ViewBase<T, FooMeta>::ViewBase;

  T foo() const {
    return T(this->meta_.foo);
  }

  int n() const {
    return this->meta_.n;
  }

  std::vector<Vector3> vectors() const {
    const int n = this->meta_.n;
    std::vector<Vector3> vs(n);
    for (int i=0; i < n; ++i) {
      auto ptr = this->holder_->Parameter(i);
      vs[i] = Eigen::Map<Vector3>(ptr);
    }
    return vs;
  }


  Result Evaluate(T t, int flags) const {
    auto r = std::make_unique<TrajectoryEvaluation<T>>();
    r->position.setZero();

    int i=0;
    for (auto&v : vectors()) {
      if (t >= T(i + 1)) {
        r->position += v;
      }
      else {
        break;
      }

      i += 1;
    }

    r->position *= foo();


    return r;
  }

  double MinTime() const override {
    return -std::numeric_limits<double>::infinity();
  }

  double MaxTime() const override {
    return std::numeric_limits<double>::infinity();
  }
};

struct SimpleMultiMeta : MetaBase {
  // References used by all calling code
  // It eiher points to external FooMeta instances or this instances __X_owned members
  FooMeta& a;
  FooMeta& b;

  SimpleMultiMeta(FooMeta& a_external, FooMeta& b_external) :
      a(a_external),
      b(b_external) { };

  SimpleMultiMeta() :
      SimpleMultiMeta(__a_owned, __b_owned) { };

  int NumParameters() const override {
    return a.NumParameters() + b.NumParameters();
  }

 private:
  // When we need to carry the data and not just a reference
  FooMeta __a_owned;
  FooMeta __b_owned;
};

template<typename T>
class SimpleMultiView : public ViewBase<T, SimpleMultiMeta> {
  using Vector3 = Eigen::Matrix<T, 3, 1>;
  using Result = std::unique_ptr<TrajectoryEvaluation<T>>;
  using BaseViewType = ViewBase<T, SimpleMultiMeta>;
 public:
  using Meta = SimpleMultiMeta;

  SimpleMultiView(std::shared_ptr<dataholders::DataHolderBase<T>> data_holder, const Meta& meta) :
      BaseViewType::ViewBase(data_holder, meta),
      view_a_(data_holder->Slice(0, meta.a.n), meta.a),
      view_b_(data_holder->Slice(meta.a.n, meta.b.n), meta.b) { };

  T AWeight() const {
    const int offset = this->meta_.a.n + this->meta_.b.n;
    return this->holder_->Parameter(offset)[0];
  }

  T BWeight() const {
    const int offset = this->meta_.a.n + this->meta_.b.n;
    return this->holder_->Parameter(offset)[1];
  }

  Result Evaluate(T t, int flags) const {
    Vector3 pos_a = view_a_.Position(t);
    Vector3 pos_b = view_b_.Position(t);

    auto r = std::make_unique<TrajectoryEvaluation<T>>();
    T wa = AWeight();
    T wb = BWeight();
    r->position = wa * pos_a + wb * pos_b + Vector3(wa, wb, T(1.));
    return r;
  }

  double MinTime() const override {
    return std::max(view_a_.MinTime(), view_b_.MinTime());
  }

  double MaxTime() const override {
    return std::min(view_a_.MaxTime(), view_b_.MaxTime());
  }

  const FooView<T> view_a_;
  const FooView<T> view_b_;
};

} // namespace detail

class FooTrajectory : public TrajectoryBase<detail::FooView> {
  using Vector3 = Eigen::Vector3d;
 public:
  static constexpr const char* CLASS_ID = "Foo";
  FooTrajectory() :
      TrajectoryBase(new dataholders::VectorHolder<double>()) { };

  void AddToProblem(ceres::Problem& problem,
                      const time_init_t &times,
                      Meta& meta,
                      std::vector<double*> &parameter_blocks,
                      std::vector<size_t> &parameter_sizes) const override {

    if (times.size() != 1) {
      throw std::length_error("Can only handle single timespans, for now");

    }

    double t1, t2;
    for (auto& tp : times) {
      t1 = tp.first;
      t2 = tp.second;
      break;
    }

    int i = 0;
    for (auto& v : vectors()) {
      if (t2 >= (i + 1)) {
        auto ptr = holder_->Parameter(i);
        size_t size = 3;
        parameter_blocks.push_back(ptr);
        parameter_sizes.push_back(size);
        problem.AddParameterBlock(ptr, size);
      }
      else {
        break;
      }

      i += 1;
    }

    meta.n = i;
  }

  double foo() const {
    return AsView().foo();
  }

  void set_foo(double x) {
    meta_.foo = x;
  }

  void AddVector(const Vector3& v) {
    size_t i = holder_->AddParameter(3);
    Eigen::Map<Vector3> vmap(holder_->Parameter(i));
    vmap = v;
    meta_.n += 1;
  }

  std::vector<Vector3> vectors() const {
    return AsView().vectors();
  }

};

namespace detail {
// Initialization object needed only to make sure that the two FooTrajectories
// are constructed fully before being passed to the SimpleMultiTrajectory.
// We must do this since the DataHolder and SimpleMultiMeta reference these.
struct SMTInit {
  std::shared_ptr<FooTrajectory> a, b;
  std::shared_ptr<dataholders::VectorHolder<double>> param_holder;
  dataholders::MultiHolder<double, 3> *holder;
  detail::SimpleMultiMeta meta;


  SMTInit() :
      a(new FooTrajectory()),
      b(new FooTrajectory()),
      param_holder(new dataholders::VectorHolder<double>()),
      holder(new dataholders::MultiHolder<double, 3>({a->Holder(),
                                                      b->Holder(),
                                                      param_holder})),
      meta(a->MetaRef(), b->MetaRef()) {};
};
} //namespace detail

class SimpleMultiTrajectory : public TrajectoryBase<detail::SimpleMultiView> {
  // Hidden constructor using SMTInit proxy object
  SimpleMultiTrajectory(const detail::SMTInit& init) :
      TrajectoryBase(init.holder, init.meta),
      foo_a(init.a),
      foo_b(init.b) {
    ParamHolder()->AddParameter(2); // a and b weight storage
    set_AWeight(1.0);
    set_BWeight(1.0);
  };

  std::shared_ptr<dataholders::MutableDataHolderBase<double>> ParamHolder() const {
    return static_cast<dataholders::MultiHolder<double, 3> *>(holder_.get())->GetHolder(2);
  }

 public:
  static constexpr const char* CLASS_ID = "SimpleMulti";
  SimpleMultiTrajectory() :
      SimpleMultiTrajectory(detail::SMTInit()) { };

  double AWeight() const {
    return AsView().AWeight();
  }

  void set_AWeight(double w) {
    ParamHolder()->Parameter(0)[0] = w;
  }

  double BWeight() const {
    return AsView().BWeight();
  }

  void set_BWeight(double w) {
    ParamHolder()->Parameter(0)[1] = w;
  }

  void AddToProblem(ceres::Problem& problem,
                      const time_init_t &times,
                      Meta& meta,
                      std::vector<double*> &parameter_blocks,
                      std::vector<size_t> &parameter_sizes) const override {
    // Trajectories
    foo_a->AddToProblem(problem, times, meta.a, parameter_blocks, parameter_sizes);
    foo_b->AddToProblem(problem, times, meta.b, parameter_blocks, parameter_sizes);

    // Weights
    auto ptr = ParamHolder()->Parameter(0);
    size_t size = 2;
    problem.AddParameterBlock(ptr, size);
    parameter_blocks.push_back(ptr);
    parameter_sizes.push_back(size);
  }

  std::shared_ptr<FooTrajectory> foo_a;
  std::shared_ptr<FooTrajectory> foo_b;
};

} // namespace trajectories
} // namespace taser

#endif //TASERV2_SIMPLE_MULTI_TRAJECTORY_H
