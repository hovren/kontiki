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

template<typename ImuModel, typename TrajectoryModel, typename T>
using ImuView = typename ImuModel::template View<TrajectoryModel, T>;

template<typename ImuModel, typename TrajectoryModel, typename T>
ImuView<ImuModel, TrajectoryModel, T> ImuMap(T const* const* params,
                                             const typename ImuModel::Meta& meta) {
  return ImuModel::template Map<TrajectoryModel, T>(params, meta);
};

namespace detail {

struct BasicImuMeta : public trajectories::MetaBase { // FIXME: Move MetaBase from trajectory.h
  int NumParameters() const override {
    return 0;
  }
};

template<typename TrajectoryModel, typename T, typename MetaType>
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

  virtual Vector3 Gyroscope(const TrajectoryView<TrajectoryModel, T> &trajectory, T t) const = 0;


 protected:
  std::shared_ptr<trajectories::dataholders::DataHolderBase<T>> holder_;
  const Meta& meta_;
};

template<typename TrajectoryModel, typename T>
class BasicImuView : public ImuViewBase<TrajectoryModel, T, BasicImuMeta> {
  using Vector3 = Eigen::Matrix<T, 3, 1>;

 public:
  using ImuViewBase<TrajectoryModel, T, BasicImuMeta>::ImuViewBase;

  Vector3 Gyroscope(const TrajectoryView<TrajectoryModel, T> &trajectory, T t) const override {
    auto result = trajectory.Evaluate(t, Flags::EvalOrientation | Flags::EvalAngularVelocity);
    // Rotate from world to body coordinate frame
    return result->orientation.conjugate()*result->angular_velocity;
  }
};

template<template<typename, typename> typename ViewTemplate, typename MetaType>
class ImuBase {
  using Vector3 = Eigen::Vector3d;
 public:
  template<typename TrajectoryModel, typename T>
  using View = ViewTemplate<TrajectoryModel, T>;
  using Meta = MetaType;

  ImuBase(trajectories::dataholders::MutableDataHolderBase<double>* holder, const Meta& meta) :
    holder_(holder),
    meta_(meta) { };

  ImuBase(trajectories::dataholders::MutableDataHolderBase<double>* holder) :
    ImuBase(holder, Meta()) { };

  template<typename TrajectoryModel>
  View<TrajectoryModel, double> AsView() const {
    return View<TrajectoryModel, double>(holder_, meta_);
  }

  template<typename TrajectoryModel, typename T>
  static View<TrajectoryModel, T> Map(T const* const* params, const Meta &meta) {
    auto ptr_holder = std::make_shared<trajectories::dataholders::PointerHolder<T>>(params);
    return View<TrajectoryModel, T>(ptr_holder, meta);
  };

  template<typename TrajectoryModel>
  Vector3 Gyroscope(std::shared_ptr<TrajectoryModel> trajectory, double t) {
    return AsView<TrajectoryModel>().Gyroscope(trajectory->AsView(), t);
  }

 protected:
  Meta meta_;
  const std::shared_ptr<trajectories::dataholders::MutableDataHolderBase<double>> holder_;
};



} // namespace detail

class BasicImu : public detail::ImuBase<detail::BasicImuView, detail::BasicImuMeta> {
 public:
  BasicImu() :
      detail::ImuBase<detail::BasicImuView, detail::BasicImuMeta>(new trajectories::dataholders::VectorHolder<double>()) { };
};

} // namespace sensors
} // namespace taser

#endif //TASERV2_IMU_H
