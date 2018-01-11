//
// Created by hannes on 2018-01-09.
//

#ifndef TASERV2_SIMPLE_MULTI_TRAJECTORY_H
#define TASERV2_SIMPLE_MULTI_TRAJECTORY_H

#include <vector>
#include <cmath>

#include "trajectory.h"
#include "../trajectory_estimator.h"

namespace taser {
namespace trajectories {

namespace detail {

struct FooMeta {
  int n;
  double foo;

  FooMeta() : n(0), foo(1.0) {  };
};

template <typename T>
class FooView : public ViewBase<T, FooView<T>, FooMeta> {
  using Result = std::unique_ptr<TrajectoryEvaluation<T>>;
  using Vector3 = Eigen::Matrix<T, 3, 1>;
 public:
  using Meta = FooMeta;
  using ViewBase<T, FooView<T>, FooMeta>::ViewBase;

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
//      std::cout << "vector["<< i <<"] = " << ptr << std::endl;
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

};

struct SimpleMultiMeta {
  // References used by all calling code
  // FIXME: Can we make these const references?
  FooMeta& a;
  FooMeta& b;

  SimpleMultiMeta(FooMeta& a_external, FooMeta& b_external) :
      a(a_external),
      b(b_external) { };

  SimpleMultiMeta() :
      SimpleMultiMeta(__a_owned, __b_owned) { };

 private:
  // When we need to carry the data and not just a reference
  FooMeta __a_owned;
  FooMeta __b_owned;
};

template<typename T>
class SimpleMultiView : public ViewBase<T, SimpleMultiView<T>, SimpleMultiMeta> {
  using Vector3 = Eigen::Matrix<T, 3, 1>;
  using Result = std::unique_ptr<TrajectoryEvaluation<T>>;
  using BaseViewType = ViewBase<T, SimpleMultiView<T>, SimpleMultiMeta>;
 public:
  using Meta = SimpleMultiMeta;

//  SimpleMultiView(T const* const* params, const Meta& meta) :
//      BaseViewType::ViewBase(params, meta),
//      view_a_(this->holder_->Slice(0, this->meta_.a.n), this->meta_.a),
//      view_b_(this->holder_->Slice(this->meta_.a.n, this->meta_.b.n), this->meta_.b) {
//  };

  SimpleMultiView(std::shared_ptr<DataHolderBase<T>> data_holder, const Meta& meta) :
      BaseViewType::ViewBase(data_holder, meta) {};
//  ,
//      view_a_(data_holder->Slice(0, meta.a.n), meta.a),
//      view_b_(data_holder->Slice(meta.a.n, meta.b.n), meta.b) {
//  };

  Result Evaluate(T t, int flags) const {
    auto dh1 = this->holder_->Slice(0, this->meta_.a.n);
    auto view_a_ = FooView<T>(dh1, this->meta_.a);
    Vector3 pos_a = view_a_.Position(t);

    auto view_b_ = FooView<T>(this->holder_->Slice(this->meta_.a.n, this->meta_.b.n), this->meta_.b);
    Vector3 pos_b = view_b_.Position(t);

    auto r = std::make_unique<TrajectoryEvaluation<T>>();
    r->position = pos_a + pos_b;
    return r;
  }

 protected:
//  const FooView<T> view_a_;
//  const FooView<T> view_b_;
};

} // namespace detail

template <typename T, int N>
class MultiHolder : public MutableDataHolderBase<T> {
 public:

  MultiHolder() {};

  MultiHolder(std::initializer_list<std::shared_ptr<MutableDataHolderBase<T>>> holder_list) {
    Initialize(holder_list);
  }

  void Initialize(std::initializer_list<std::shared_ptr<MutableDataHolderBase<T>>> holder_list) {
    if(holder_list.size() != N) {
      throw std::length_error("Wrong number of arguments");
    }

    int i=0;
    for (auto h : holder_list) {
      holders_[i] = h;
      i += 1;
    }
  }

  T* Parameter(size_t i) const override {
    int j = 0;
    for (auto h : holders_) {
      const size_t n = h->Size();
      if ((j + n) > i) {
        return h->Parameter(i);
      }
      else {
        j += n;
      }
    }

    throw std::length_error("Parameter index out of range");
  }

  std::shared_ptr<DataHolderBase<T>> Slice(size_t start, size_t size) const override {
//    std::cout << "Slice: " << start << " size " << size << std::endl;
//    std::cout << "Subholder sizes: ";
//    for (auto h : holders_) {
//      std::cout << h->Size() << " ";
//    }
//    std::cout << std::endl;
//
    int j = 0;
    for (auto h : holders_) {
      const size_t n = h->Size();
      // Only accept slices on exact boundaries
      if (j == start) {
        return h;
      }
      else if ((j + n) <= start) {
        j += n;
      }
      else {
        break;
      }
    }

    throw std::runtime_error("Can only slice on exact holder boundaries");
  }

  size_t AddParameter(size_t ndims) override {
    throw std::runtime_error("Use the specific MultiHolder interface instead!");
  }

  size_t Size() const override {
    size_t sz = 0;
    for (auto h : holders_) {
      sz += h->Size();
    }

    return sz;
  }

  std::shared_ptr<MutableDataHolderBase<T>> GetHolder(size_t i) {
    return holders_[i];
  }

 protected:
  std::array<std::shared_ptr<MutableDataHolderBase<T>>, N> holders_; // FIXME: Ownage of the holder pointers is unclear
};

class FooTrajectory : public TrajectoryBase<detail::FooView> {
  using Vector3 = Eigen::Vector3d;
 public:
  static constexpr const char* CLASS_ID = "Foo";
  FooTrajectory() :
      TrajectoryBase(new VectorHolder<double>()) { };

  void AddToProblem(ceres::Problem& problem,
                      const time_init_t &times,
                      Meta& meta,
                      std::vector<double*> &parameter_blocks,
                      std::vector<size_t> &parameter_sizes) {

    if (times.size() != 1) {
      throw std::length_error("Can only handle single timespans, for now");

    }

    double t1, t2;
    for (auto& tp : times) {
      t1 = tp.first;
      t2 = tp.second;
      break;
    }

    // We must have all vectors up to t2
    std::cout << "FooTraj extract times " << t1 << " to " << t2 << std::endl;

    int i = 0;
    for (auto& v : vectors()) {
      if (t2 >= (i + 1)) {
        auto ptr = holder_->Parameter(i);
        std::cout << "Adding i=" << i << " ptr=" << ptr << " v=" << v.transpose() << std::endl;
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

struct SMTInit {
  std::shared_ptr<FooTrajectory> a, b;
  MultiHolder<double, 2>* holder;
  detail::SimpleMultiMeta meta;

  SMTInit() :
      a(new FooTrajectory()),
      b(new FooTrajectory()),
      holder(new MultiHolder<double, 2>({a->Holder(), b->Holder()})),
      meta(a->MetaRef(), b->MetaRef()) { };
};

class SimpleMultiTrajectory : public TrajectoryBase<detail::SimpleMultiView> {
 public:
  static constexpr const char* CLASS_ID = "SimpleMulti";
  using HolderType = MultiHolder<double, 2>;

  SimpleMultiTrajectory(const SMTInit& init) :
      TrajectoryBase(init.holder, init.meta),
      foo_a(init.a),
      foo_b(init.b) { };

  SimpleMultiTrajectory() :
      SimpleMultiTrajectory(SMTInit()) { };

//  SimpleMultiTrajectory() :
//      TrajectoryBase(new HolderType()) {
////                                           detail::SimpleMultiMeta(foo_a.MetaRef(), foo_b.MetaRef())) {
//    static_cast<HolderType*>(this->holder_.get())->Initialize({foo_a.Holder(), foo_b.Holder()});
//    std::cout << "SimpleMultiTrajectory()" << std::endl;
//    std::cout << "a meta " << &foo_a.MetaRef() << std::endl;
//    std::cout << "b meta " << &foo_b.MetaRef() << std::endl;
//  };

  void AddToProblem(ceres::Problem& problem,
                      const time_init_t &times,
                      Meta& meta,
                      std::vector<double*> &parameter_blocks,
                      std::vector<size_t> &parameter_sizes) {
    std::cout << "SimpleMultiTrajectory::AddToProblem()" << std::endl;
    std::cout << "BEFORE nparams=" << parameter_blocks.size() << std::endl;
    foo_a->AddToProblem(problem, times, meta.a, parameter_blocks, parameter_sizes);
    foo_b->AddToProblem(problem, times, meta.b, parameter_blocks, parameter_sizes);
    std::cout << "AFTER nparams=" << parameter_blocks.size() << std::endl;

    std::cout << "a.n=" << meta.a.n << " b.n=" << meta.b.n << std::endl;

    if (parameter_blocks.size() != parameter_sizes.size()) {
      throw std::length_error("Num blocks did not match num sizes");
    }

    for (int i=0; i < parameter_blocks.size(); ++i) {
      std::cout << i << ": ptr=" << parameter_blocks[i] << " size=" << parameter_sizes[i] << std::endl;
    }

  }

  std::shared_ptr<FooTrajectory> foo_a;
  std::shared_ptr<FooTrajectory> foo_b;
};

} // namespace trajectories
} // namespace taser

#endif //TASERV2_SIMPLE_MULTI_TRAJECTORY_H
