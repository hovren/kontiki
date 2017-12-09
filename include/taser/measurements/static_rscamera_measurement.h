//
// Created by hannes on 2017-12-04.
//

#ifndef TASERV2_STATIC_RSCAMERA_MEASUREMENT_H
#define TASERV2_STATIC_RSCAMERA_MEASUREMENT_H

#include "trajectories/trajectory.h"
#include "sfm/observation.h"
#include "sfm/landmark.h"
#include "sfm/view.h"
#include "trajectory_estimator.h"
#include "cameras/camera.h"

namespace TT = taser::trajectories;

namespace taser {
namespace measurements {

template<
    typename T,
    template<typename> typename TrajectoryModel,
    typename CameraModel>
Eigen::Matrix<T, 2, 1> reproject_static(const Observation& ref, const Observation& obs, T inverse_depth,
                                 const TrajectoryModel<T>& trajectory, const CameraModel& camera) {
  using Vector3 = Eigen::Matrix<T, 3, 1>;
  using Vector2 = Eigen::Matrix<T, 2, 1>;

  T row_delta = T(camera.readout() / camera.rows());
  T t_ref = ref.view()->t0() + ref.v() * row_delta;
  T t_obs = obs.view()->t0() + obs.v() * row_delta;

  auto eval_ref = trajectory.Evaluate(t_ref, TT::EvalPosition | TT::EvalOrientation);
  auto eval_obs = trajectory.Evaluate(t_obs, TT::EvalPosition | TT::EvalOrientation);

  const taser::cameras::RelativePose &rel_pose = camera.relative_pose();
  const Vector3 p_ct = rel_pose.translation.cast<T>();
  const Eigen::Quaternion<T> q_ct = rel_pose.orientation.cast<T>();

  Vector2 y = ref.uv().cast<T>();
  Vector3 yh = camera.Unproject(y);

  // fixme: It might be faster to make this a single expression (Eigen optimizations)
  Vector3 X_ref = q_ct.conjugate() * (yh - inverse_depth * p_ct);
  Vector3 X = eval_ref->orientation * X_ref + eval_ref->position * inverse_depth;
  Vector3 X_obs = eval_obs->orientation.conjugate() * (X - inverse_depth * eval_obs->position);
  Vector3 X_camera = q_ct * X_obs + p_ct * inverse_depth;

  return camera.Project(X_camera);
}


  template<typename CameraImpl>
  class StaticRsCameraMeasurement {
    using Vector2 = Eigen::Vector2d;
    using ThisType = StaticRsCameraMeasurement<CameraImpl>;
   public:
    StaticRsCameraMeasurement(std::shared_ptr<CameraImpl> camera, std::shared_ptr<Landmark> landmark, std::shared_ptr<Observation> obs)
        : camera(camera), landmark(landmark), observation(obs) {};

    std::shared_ptr<CameraImpl> camera;
    // Measurement data
    std::shared_ptr<taser::Landmark> landmark;
    std::shared_ptr<taser::Observation> observation;

    template<typename T, template<typename> typename TrajectoryModel>
    Eigen::Matrix<T, 2, 1> Project(const TrajectoryModel<T> &trajectory) const {
      return reproject_static(*landmark->reference(), *observation, T(landmark->inverse_depth()), trajectory, *camera);
    };

    template<typename T, template<typename> typename TrajectoryModel>
    Eigen::Matrix<T, 2, 1> error(const TrajectoryModel<T> &trajectory) const {
      Eigen::Matrix<T,2,1> y_hat = this->Project(trajectory);
      return observation->uv().cast<T>() - y_hat;
    }

   protected:

    template<template<typename> typename TrajectoryModel>
    struct Residual {
      Residual(const ThisType &m) : measurement(m) {};

      template <typename T>
      bool operator()(T const* const* params, T* residual) const {
        auto trajectory = TrajectoryModel<T>::Unpack(params, meta);
        Eigen::Map<Eigen::Matrix<T,2,1>> r(residual);
        r = measurement.error(trajectory);
        return true;
      }

      const ThisType &measurement;
      typename TrajectoryModel<double>::Meta meta;
    }; // Residual;

    template<template<typename> typename TrajectoryModel>
    void AddToEstimator(taser::TrajectoryEstimator<TrajectoryModel>& estimator) {
      using ResidualImpl = Residual<TrajectoryModel>;
      auto residual = new ResidualImpl(*this);
      auto cost_function = new ceres::DynamicAutoDiffCostFunction<ResidualImpl>(residual);
      std::vector<double*> parameter_blocks;
      std::vector<size_t> parameter_sizes;

      // Add trajectory to problem
      auto t0_ref = landmark->reference()->view()->t0();
      auto t0_obs = observation->view()->t0();
      estimator.AddTrajectoryForTimes({
                                          {t0_ref, t0_ref + camera->readout()},
                                          {t0_obs, t0_obs + camera->readout()}
                                      },
                                      residual->meta, parameter_blocks, parameter_sizes);
      for (auto ndims : parameter_sizes) {
        cost_function->AddParameterBlock(ndims);
      }

      // Add measurement info
      cost_function->SetNumResiduals(2);

      // Landmark inverse depth is the only extra parameter
      double *p_rho = landmark->inverse_depth_ptr();
      estimator.problem().AddParameterBlock(p_rho, 1);
      parameter_blocks.push_back(p_rho);
      cost_function->AddParameterBlock(1);

      // Give residual block to Problem
      estimator.problem().AddResidualBlock(cost_function, nullptr, parameter_blocks);
    }

    template<template<typename> typename TrajectoryModel>
    friend class taser::TrajectoryEstimator;
  };
} // namespace measurements
} // namespace taser

#endif //TASERV2_STATIC_RSCAMERA_MEASUREMENT_H
