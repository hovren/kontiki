#ifndef TASERV2_SPLIT_TRAJECTORY_H
#define TASERV2_SPLIT_TRAJECTORY_H

#include "uniform_r3_spline_trajectory.h"
#include "uniform_so3_spline_trajectory.h"
#include "dataholders/multiholder.h"

namespace taser {
namespace trajectories {
namespace detail {

struct SplitMeta : public MetaBase {
  using R3Meta = UniformR3SplineTrajectory::Meta;
  using SO3Meta = UniformSO3SplineTrajectory::Meta;

  // References either to the child trajectory metas, or to one created when adding a residual to a problem
  R3Meta &r3_meta;
  SO3Meta &so3_meta;

  SplitMeta(R3Meta &r3_meta_external, SO3Meta &so3_meta_external) :
      r3_meta(r3_meta_external),
      so3_meta(so3_meta_external) { };

  SplitMeta() :
      SplitMeta(__r3_meta_owned, __so3_meta_owned) { };

  int NumParameters() const override {
    return r3_meta.NumParameters() + so3_meta.NumParameters();
  }

 private:
  // When creating a view, we need to keep the meta somewhere
  R3Meta __r3_meta_owned;
  SO3Meta __so3_meta_owned;
};


template<typename T>
class SplitView : public ViewBase<T, SplitMeta> {
  using Result = std::unique_ptr<TrajectoryEvaluation<T>>;
  using BaseType = ViewBase<T, SplitMeta>;
 public:
  using Meta = SplitMeta;

  SplitView(std::shared_ptr<dataholders::DataHolderBase<T>> data_holder, const Meta& meta) :
    BaseType::ViewBase(data_holder, meta),
    r3_view_(data_holder->Slice(0, meta.r3_meta.NumParameters()), meta.r3_meta),
    so3_view_(data_holder->Slice(meta.r3_meta.NumParameters(), meta.so3_meta.NumParameters()), meta.so3_meta) { };

  Result Evaluate(T t, int flags) const {
    Result result = std::make_unique<TrajectoryEvaluation<T>>(flags);

    if (result->needs.AnyLinear()) {
      Result r3_result = r3_view_.Evaluate(t, flags);
      result->position = r3_result->position;
      result->velocity = r3_result->velocity;
      result->acceleration = r3_result->acceleration;
    }

    if (result->needs.AnyRotation()) {
      Result so3_result = so3_view_.Evaluate(t, flags);
      result->orientation = so3_result->orientation;
      result->angular_velocity = so3_result->angular_velocity;
    }

    return result;
  }

  double MinTime() const override {
    return std::max<double>(r3_view_.MinTime(), so3_view_.MinTime());
  }

  double MaxTime() const override {
    return std::min<double>(r3_view_.MaxTime(), so3_view_.MaxTime());
  }

  const UniformR3SplineTrajectory::View<T> r3_view_;
  const UniformSO3SplineTrajectory::View<T> so3_view_;
};


// Initialization object needed only to make sure that the R3 and SO3 splines
// are constructed fully before being passed to the SplitTrajectory.
// We must do this since the DataHolder and SplitMeta reference these.
struct SplitInit {
  std::shared_ptr<UniformR3SplineTrajectory> r3_trajectory;
  std::shared_ptr<UniformSO3SplineTrajectory> so3_trajectory;
  std::shared_ptr<dataholders::VectorHolder<double>> param_holder;
  dataholders::MultiHolder<double, 3> *holder;
  SplitMeta meta;


  // FIXME: Depend on other constructor
  SplitInit(double r3_dt, double so3_dt, double r3_t0, double so3_t0) :
      r3_trajectory(new UniformR3SplineTrajectory(r3_dt, r3_t0)),
      so3_trajectory(new UniformSO3SplineTrajectory(so3_dt, so3_t0)),
      param_holder(new dataholders::VectorHolder<double>()),
      holder(new dataholders::MultiHolder<double, 3>({r3_trajectory->Holder(),
                                                      so3_trajectory->Holder(),
                                                      param_holder})),
      meta(r3_trajectory->MetaRef(), so3_trajectory->MetaRef()) {};

  SplitInit(std::shared_ptr<UniformR3SplineTrajectory> r3_traj, std::shared_ptr<UniformSO3SplineTrajectory> so3_traj) :
      r3_trajectory(r3_traj),
      so3_trajectory(so3_traj),
      param_holder(new dataholders::VectorHolder<double>()),
      holder(new dataholders::MultiHolder<double, 3>({r3_trajectory->Holder(),
                                                      so3_trajectory->Holder(),
                                                      param_holder})),
      meta(r3_trajectory->MetaRef(), so3_trajectory->MetaRef()) {};

};

} // namespace detail

class SplitTrajectory : public TrajectoryBase<detail::SplitView> {
  // Hidden constructors
  SplitTrajectory(const detail::SplitInit &init) :
      TrajectoryBase(init.holder, init.meta),
      r3_trajectory_(init.r3_trajectory),
      so3_trajectory_(init.so3_trajectory) { };
 public:
  static constexpr const char* CLASS_ID = "Split";

  SplitTrajectory(std::shared_ptr<UniformR3SplineTrajectory> r3_traj,
                  std::shared_ptr<UniformSO3SplineTrajectory> so3_traj) :
    SplitTrajectory(detail::SplitInit(r3_traj, so3_traj)) { };

  SplitTrajectory(double r3_dt, double so3_dt, double r3_t0, double so3_t0) :
      SplitTrajectory(detail::SplitInit(r3_dt, so3_dt, r3_t0, so3_t0)) { };

  SplitTrajectory(double r3_dt, double so3_dt) :
      SplitTrajectory(r3_dt, so3_dt, 0.0, 0.0) { };

  SplitTrajectory() :
      SplitTrajectory(1.0, 1.0) { };

  std::shared_ptr<UniformR3SplineTrajectory> R3Spline() const {
    return r3_trajectory_;
  }

  std::shared_ptr<UniformSO3SplineTrajectory> SO3Spline() const {
    return so3_trajectory_;
  }

  void AddToProblem(ceres::Problem& problem,
                    const time_init_t &times,
                    Meta& meta,
                    std::vector<double*> &parameter_blocks,
                    std::vector<size_t> &parameter_sizes) const override {
    // Pass
  }

 protected:
  std::shared_ptr<UniformR3SplineTrajectory> r3_trajectory_;
  std::shared_ptr<UniformSO3SplineTrajectory> so3_trajectory_;
};

} // namespace trajectories
} // namespace taser

#endif //TASERV2_SPLIT_TRAJECTORY_H
