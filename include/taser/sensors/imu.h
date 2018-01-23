#ifndef TASERV2_IMU_H
#define TASERV2_IMU_H

#include <Eigen/Dense>

#include <taser/trajectories/dataholders/dataholder.h>
#include <taser/trajectories/dataholders/vectorholder.h>
#include <taser/trajectories/trajectory.h>

namespace taser {
namespace sensors {

using Flags = trajectories::EvaluationFlags;
using trajectories::TrajectoryView;
using trajectories::TrajectoryMap;
using trajectories::time_span_t;
using trajectories::time_init_t;

template<typename ImuModel, typename T>
using ImuView = typename ImuModel::template View<T>;

template<typename ImuModel, typename T>
ImuView<ImuModel, T> ImuMap(T const* const* params,
                            const typename ImuModel::Meta& meta) {
  return ImuModel::template Map<T>(params, meta);
};

namespace detail {

struct BasicImuMeta : public trajectories::MetaBase { // FIXME: Move MetaBase from trajectory.h
  int NumParameters() const override {
    return 0;
  }
};

template<typename T, typename MetaType>
class ImuViewBase {
  static_assert(
      std::is_base_of<trajectories::MetaBase, MetaType>::value,
      "Meta must be subclass of MetaBase"
  );

  using Vector3 = Eigen::Matrix<T, 3, 1>;
 public:
  using Meta = MetaType;

  ImuViewBase(std::shared_ptr<trajectories::dataholders::DataHolderBase<T>> data_holder, const Meta& meta) :
      meta_(meta), holder_(data_holder) { };

  // Standard Gyroscope function
  template<typename TrajectoryModel>
  Vector3 DefaultGyroscope(const TrajectoryView<TrajectoryModel, T> &trajectory, T t) const {
    auto result = trajectory.Evaluate(t, Flags::EvalOrientation | Flags::EvalAngularVelocity);
    // Rotate from world to body coordinate frame
    return result->orientation.conjugate()*result->angular_velocity;
  }

  template<typename TrajectoryModel>
  Vector3 Gyroscope(const TrajectoryView<TrajectoryModel, T> &trajectory, T t) const {
    return DefaultGyroscope<TrajectoryModel>(trajectory, t);
  }

 protected:
  std::shared_ptr<trajectories::dataholders::DataHolderBase<T>> holder_;
  const Meta& meta_;
};

template<typename T>
class BasicImuView : public ImuViewBase<T, BasicImuMeta> {
  using Vector3 = Eigen::Matrix<T, 3, 1>;

 public:
  using ImuViewBase<T, BasicImuMeta>::ImuViewBase;

  // We could override (redefine) the Gyroscope and Accelerometer template methods here
  // If we don't, the methods in ImuViewBase will be used

//  template<typename TrajectoryModel>
//  Vector3 Gyroscope(const TrajectoryView<TrajectoryModel, T> &trajectory, T t) const {
//    Vector3 base_gyro = this->template DefaultGyroscope<TrajectoryModel>(trajectory, t);
//    return base_gyro + Vector3(666, 666, 666);
//  }
};

template<template<typename> typename ViewTemplate>
class ImuBase {
  using Vector3 = Eigen::Vector3d;
 public:
  template<typename T>
  using View = ViewTemplate<T>;
  using Meta = typename View<double>::Meta;

  ImuBase(trajectories::dataholders::MutableDataHolderBase<double>* holder, const Meta& meta) :
    holder_(holder),
    meta_(meta) { };

  ImuBase(trajectories::dataholders::MutableDataHolderBase<double>* holder) :
    ImuBase(holder, Meta()) { };

  View<double> AsView() const {
    return View<double>(holder_, meta_);
  }

  template<typename T>
  static View<T> Map(T const* const* params, const Meta &meta) {
    auto ptr_holder = std::make_shared<trajectories::dataholders::PointerHolder<T>>(params);
    return View<T>(ptr_holder, meta);
  };

  template<typename TrajectoryModel>
  Vector3 Gyroscope(std::shared_ptr<TrajectoryModel> trajectory, double t) const {
    return AsView().template Gyroscope<TrajectoryModel>(trajectory->AsView(), t);
  }

  // Add trajectory to problem for a set of time spans
  // Implementers can assume that the the list of time spans is ordered
  virtual void AddToProblem(ceres::Problem& problem,
                            const time_init_t &times,
                            Meta& meta,
                            std::vector<double*> &parameter_blocks,
                            std::vector<size_t> &parameter_sizes) const {
    // By default, do nothing
  }

 protected:
  Meta meta_;
  const std::shared_ptr<trajectories::dataholders::MutableDataHolderBase<double>> holder_;
};

} // namespace detail

class BasicImu : public detail::ImuBase<detail::BasicImuView> {
 public:
  BasicImu() :
      detail::ImuBase<detail::BasicImuView>(new trajectories::dataholders::VectorHolder<double>()) { };
};

} // namespace sensors
} // namespace taser

#endif //TASERV2_IMU_H
