//
// Created by hannes on 2018-01-09.
//

#ifndef TASERV2_SIMPLE_MULTI_TRAJECTORY_H
#define TASERV2_SIMPLE_MULTI_TRAJECTORY_H

#include "trajectory.h"
#include "../trajectory_estimator.h"

namespace taser {
namespace trajectories {

namespace detail {

struct FooMeta {
  int n;
  double foo;

  FooMeta() : n(0), foo(1.0) { std::cout << "FooMeta() " << this << std::endl; };
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

  std::vector<Vector3> vectors() const {
    const int n = this->meta_.n;
    std::vector<Vector3> vs(n);
    for (int i=0; i < n; ++i) {
      vs[i] = Eigen::Map<Vector3>(this->holder_->Parameter(i));
    }
    return vs;
  }

  Result Evaluate(T t, int flags) const {
    auto r = std::make_unique<TrajectoryEvaluation<T>>();
    r->position.setZero();
    for (Vector3& v : vectors()) {
      r->position += v;
    }

    r->position *= foo();


    return r;
  }

};

struct SimpleMultiMeta {
  // References used by all calling code
  const FooMeta& a;
  const FooMeta& b;

  SimpleMultiMeta(const FooMeta& a_external, const FooMeta& b_external) : a(a_external), b(b_external) { std::cout << "SimpleMultiMeta()" << std::endl; };
  SimpleMultiMeta() : SimpleMultiMeta(__a_owned, __b_owned) { };

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

  SimpleMultiView(T const* const* params, const Meta& meta) :
      BaseViewType::ViewBase(params, meta),
      view_a_(this->holder_->Slice(0, this->meta_.a.n).get(), this->meta_.a),
      view_b_(this->holder_->Slice(this->meta_.a.n, this->meta_.b.n).get(), this->meta_.b) {};

  SimpleMultiView(DataHolderBase<T>* data_holder, const Meta& meta) :
      BaseViewType::ViewBase(data_holder, meta),
      view_a_(this->holder_->Slice(0, this->meta_.a.n).get(), this->meta_.a),
      view_b_(this->holder_->Slice(this->meta_.a.n, this->meta_.b.n).get(), this->meta_.b) {};

  Result Evaluate(T t, int flags) const {
    Vector3 pos_a = view_a_.Position(t);
    Vector3 pos_b = view_b_.Position(t);
    auto r = std::make_unique<TrajectoryEvaluation<T>>();
    r->position = pos_a + pos_b;
    return r;
  }

 protected:
  const FooView<T> view_a_;
  const FooView<T> view_b_;
};

} // namespace detail

template <typename T, int N>
class MultiHolder : public MutableDataHolderBase<T> {
 public:

  MultiHolder() {};

  MultiHolder(std::initializer_list<MutableDataHolderBase<T>*> holder_list) {
    std::cout << "MultiHolder constructor" << std::endl;
    Initialize(holder_list);
  }

  void Initialize(std::initializer_list<MutableDataHolderBase<T>*> holder_list) {
    if(holder_list.size() != N) {
      throw std::length_error("Wrong number of arguments");
    }

    int i=0;
    for (auto h : holder_list) {
      holders_[i].reset(h); //std::shared_ptr<DataHolderBase<T>>(h);
      i += 1;
    }

    std::cout << "Initialize finished with " << holder_list.size() << " input holders " << std::endl;
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
  FooTrajectory() : TrajectoryBase(new VectorHolder<double>()) {
    std::cout << "FooTrajectory()" << std::endl;
  };

  void AddToEstimator(TrajectoryEstimator<FooTrajectory> &estimator,
                      const time_init_t &times,
                      Meta& meta,
                      std::vector<double*> &parameter_blocks,
                      std::vector<size_t> &parameter_sizes) {
    // Do nothing yet
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
      meta(a->MetaRef(), b->MetaRef())
      {
        std::cout << "SMTInit()" << std::endl;
        std::cout << "meta.a=" << &meta.a << " meta.b=" << &meta.b << std::endl;
        std::cout << "a.holder=" << a->Holder() << " holder[0]=" << holder->GetHolder(0) << std::endl;
        std::cout << "b.holder=" << b->Holder() << " holder[1]=" << holder->GetHolder(1) << std::endl;

      };
};

class SimpleMultiTrajectory : public TrajectoryBase<detail::SimpleMultiView> {
 public:
  static constexpr const char* CLASS_ID = "SimpleMulti";
  using HolderType = MultiHolder<double, 2>;

  SimpleMultiTrajectory(const SMTInit& init) :
      TrajectoryBase(init.holder, init.meta),
      foo_a(init.a),
      foo_b(init.b) {
    std::cout << "SimpleMultiTrajectory() with init" << std::endl;
    std::cout << "foo_a.meta=" << &foo_a->MetaRef() << " meta.a=" << &meta_.a << std::endl;
    std::cout << "foo_b.meta=" << &foo_b->MetaRef() << " meta.b=" << &meta_.b << std::endl;
    std::cout << "foo_a.holder=" << foo_a->Holder() << " holder[0]=" << static_cast<HolderType*>(holder_.get())->GetHolder(0) << std::endl;
    std::cout << "foo_b.holder=" << foo_b->Holder() << " holder[1]=" << static_cast<HolderType*>(holder_.get())->GetHolder(1) << std::endl;
  };

  SimpleMultiTrajectory() :
      SimpleMultiTrajectory(SMTInit())
  {std::cout << "SimpleMultiTrajectory() without init" << std::endl; };

//  SimpleMultiTrajectory() :
//      TrajectoryBase(new HolderType()) {
////                                           detail::SimpleMultiMeta(foo_a.MetaRef(), foo_b.MetaRef())) {
//    static_cast<HolderType*>(this->holder_.get())->Initialize({foo_a.Holder(), foo_b.Holder()});
//    std::cout << "SimpleMultiTrajectory()" << std::endl;
//    std::cout << "a meta " << &foo_a.MetaRef() << std::endl;
//    std::cout << "b meta " << &foo_b.MetaRef() << std::endl;
//  };

  void AddToEstimator(TrajectoryEstimator<SimpleMultiTrajectory> &estimator,
                      const time_init_t &times,
                      Meta& meta,
                      std::vector<double*> &parameter_blocks,
                      std::vector<size_t> &parameter_sizes) {
    // Do nothing yet
  }

  std::shared_ptr<FooTrajectory> foo_a;
  std::shared_ptr<FooTrajectory> foo_b;
};

} // namespace trajectories
} // namespace taser

#endif //TASERV2_SIMPLE_MULTI_TRAJECTORY_H
