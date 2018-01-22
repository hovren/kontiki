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
 public:
  using Meta = SplitMeta;
  using ViewBase<T, SplitMeta>::ViewBase;

  Result Evaluate(T t, int flags) const {
    Result result = std::make_unique<TrajectoryEvaluation<T>>();

    return result;
  }

  double MinTime() const override {
    return 0;
  }

  double MaxTime() const override {
    return 0;
  }

//  const UniformR3SplineTrajectory::View<T> r3_view_;
//  const UniformSO3SplineTrajectory::View<T> so3_view_;
};


// Initialization object needed only to make sure that the two FooTrajectories
// are constructed fully before being passed to the SimpleMultiTrajectory.
// We must do this since the DataHolder and SimpleMultiMeta reference these.
struct SplitInit {
  std::shared_ptr<UniformR3SplineTrajectory> r3_trajectory;
  std::shared_ptr<UniformSO3SplineTrajectory> so3_trajectory;
  std::shared_ptr<dataholders::VectorHolder<double>> param_holder;
  dataholders::MultiHolder<double, 3> *holder;
  SplitMeta meta;


  SplitInit() :
      r3_trajectory(new UniformR3SplineTrajectory()),
      so3_trajectory(new UniformSO3SplineTrajectory()),
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

  SplitTrajectory() :
    SplitTrajectory(detail::SplitInit()) { };

  void AddToProblem(ceres::Problem& problem,
                    const time_init_t &times,
                    Meta& meta,
                    std::vector<double*> &parameter_blocks,
                    std::vector<size_t> &parameter_sizes) const override {
    // Pass
  }

  std::shared_ptr<UniformR3SplineTrajectory> r3_trajectory_;
  std::shared_ptr<UniformSO3SplineTrajectory> so3_trajectory_;
};

} // namespace trajectories
} // namespace taser

#endif //TASERV2_SPLIT_TRAJECTORY_H
