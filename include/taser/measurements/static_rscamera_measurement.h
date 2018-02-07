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
#include "taser/sensors/camera.h"
#include <taser/types.h>

namespace taser {
namespace measurements {

template<
    typename TrajectoryModel,
    typename CameraModel,
    typename T>
Eigen::Matrix<T, 2, 1> reproject_static(const Observation& ref,
                                        const Observation& obs,
                                        T inverse_depth,
                                        const type::Trajectory<TrajectoryModel, T>& trajectory,
                                        const type::Camera<CameraModel, T>& camera) {
  using Vector3 = Eigen::Matrix<T, 3, 1>;
  using Vector2 = Eigen::Matrix<T, 2, 1>;
  using Flags = trajectories::EvaluationFlags;

  T row_delta = camera.readout() / T(camera.rows());
  T t_ref = ref.view()->t0() + ref.v() * row_delta;
  T t_obs = obs.view()->t0() + obs.v() * row_delta;

  int flags = Flags::EvalPosition | Flags::EvalOrientation;
  auto eval_ref = trajectory.Evaluate(t_ref, flags);
  auto eval_obs = trajectory.Evaluate(t_obs, flags);

  // FIXME: We have a ToTrajectory/FromTrajectory function
  const taser::sensors::RelativePose<T> &rel_pose = camera.relative_pose();
  const Vector3 p_ct = rel_pose.translation;
  const Eigen::Quaternion<T> q_ct = rel_pose.orientation;

  Vector2 y = ref.uv().cast<T>();
  Vector3 yh = camera.Unproject(y);

//  std::cout << "y=" << y.transpose() << "\nyh=" << yh.transpose() << std::endl;

  // fixme: It might be faster to make this a single expression (Eigen optimizations)
  Vector3 X_ref = q_ct.conjugate() * (yh - inverse_depth * p_ct);
  Vector3 X = eval_ref->orientation * X_ref + eval_ref->position * inverse_depth;
  Vector3 X_obs = eval_obs->orientation.conjugate() * (X - inverse_depth * eval_obs->position);
  Vector3 X_camera = q_ct * X_obs + p_ct * inverse_depth;

  return camera.Project(X_camera);
}


  template<typename CameraModel>
  class StaticRsCameraMeasurement {
    using Vector2 = Eigen::Vector2d;
   public:
    StaticRsCameraMeasurement(std::shared_ptr<CameraModel> camera, std::shared_ptr<Observation> obs, double huber_loss)
        : camera(camera), observation(obs), loss_function_(huber_loss) {};

    StaticRsCameraMeasurement(std::shared_ptr<CameraModel> camera, std::shared_ptr<Observation> obs)
        : StaticRsCameraMeasurement(camera, obs, 5.) { };

    std::shared_ptr<CameraModel> camera;
    // Measurement data
    std::shared_ptr<taser::Observation> observation;

    template<typename TrajectoryModel, typename T>
    Eigen::Matrix<T, 2, 1> Project(const type::Trajectory<TrajectoryModel, T> &trajectory,
                                   const type::Camera<CameraModel, T> &camera,
                                   const T inverse_depth) const {
      return reproject_static<TrajectoryModel, CameraModel>(*observation->landmark()->reference(), *observation, inverse_depth, trajectory, camera);
    };

    template<typename TrajectoryModel>
    Vector2 Project(const type::Trajectory<TrajectoryModel, double> &trajectory) const {
      return Project<TrajectoryModel, double>(trajectory, *camera, observation->landmark()->inverse_depth());
    };

    template<typename TrajectoryModel, typename T>
    Eigen::Matrix<T, 2, 1> Error(const type::Trajectory<TrajectoryModel, T> &trajectory,
                                 const type::Camera<CameraModel, T> &camera,
                                 const T inverse_depth) const {
      Eigen::Matrix<T,2,1> y_hat = this->Project<TrajectoryModel, T>(trajectory, camera, inverse_depth);
      return observation->uv().cast<T>() - y_hat;
    }

    template<typename TrajectoryModel>
    Vector2 Error(const type::Trajectory<TrajectoryModel, double> &trajectory) const {
      return Error<TrajectoryModel, double>(trajectory, *camera, observation->landmark()->inverse_depth());
    };

    template<typename TrajectoryModel>
    Vector2 Measure(const type::Trajectory<TrajectoryModel, double> &trajectory) const {
      return Project<TrajectoryModel, double>(trajectory, *camera, observation->landmark()->inverse_depth());
    };

   protected:

    template<typename TrajectoryModel>
    struct Residual {
      Residual(const StaticRsCameraMeasurement<CameraModel> &m) : measurement(m) {};

      template <typename T>
      bool operator()(T const* const* params, T* residual) const {
        size_t offset = 0;
        auto trajectory = entity::Map<TrajectoryModel, T>(&params[offset], trajectory_meta);
        offset += trajectory_meta.NumParameters();
        auto camera = entity::Map<CameraModel, T>(&params[offset], camera_meta);
        offset += camera_meta.NumParameters();
        T inverse_depth = params[offset][0];
        Eigen::Map<Eigen::Matrix<T,2,1>> r(residual);
        r = measurement.Error<TrajectoryModel, T>(trajectory, camera, inverse_depth);
        return true;
      }

      const StaticRsCameraMeasurement<CameraModel> &measurement;
      typename TrajectoryModel::Meta trajectory_meta;
      typename CameraModel::Meta camera_meta;
    }; // Residual;

    template<typename TrajectoryModel>
    void AddToEstimator(taser::TrajectoryEstimator<TrajectoryModel>& estimator) {
      using ResidualImpl = Residual<TrajectoryModel>;
      auto residual = new ResidualImpl(*this);
      auto cost_function = new ceres::DynamicAutoDiffCostFunction<ResidualImpl>(residual);
      std::vector<entity::ParameterInfo<double>> parameters;

      // Find timespans for the two observations
      const auto landmark = observation->landmark();
      auto t0_ref = landmark->reference()->view()->t0();
      auto t0_obs = observation->view()->t0();

      // The list of spans must be ordered
      double t1, t2;
      if (t0_ref <= t0_obs) {
        t1 = t0_ref;
        t2 = t0_obs;
      }
      else {
        t1 = t0_obs;
        t2 = t0_ref;
      }

      estimator.AddTrajectoryForTimes({
                                          {t1, t1 + camera->readout()},
                                          {t2, t2 + camera->readout()}
                                      },
                                      residual->trajectory_meta,
                                      parameters);

      // Add camera to proble
      camera->AddToProblem(estimator.problem(), {
                               {t1, t1 + camera->readout()},
                               {t2, t2 + camera->readout()}
                           },
                           residual->camera_meta,
                           parameters);

      // Add Landmark inverse depth
      // FIXME: Landmarks could be an entity
      double *p_rho = landmark->inverse_depth_ptr();
      estimator.problem().AddParameterBlock(p_rho, 1);
      estimator.problem().SetParameterLowerBound(p_rho, 0, 0.); // Only positive inverse depths please
      parameters.push_back(entity::ParameterInfo<double>(p_rho, 1));

      // Add parameters to cost function
      for (auto& pi : parameters) {
        cost_function->AddParameterBlock(pi.size);
      }

      // Add measurement info
      cost_function->SetNumResiduals(2);

      // Give residual block to Problem
      estimator.problem().AddResidualBlock(cost_function,
                                           &loss_function_,
                                           entity::ParameterInfo<double>::ToParameterBlocks(parameters));
    }

    // The loss function is not a pointer since the Problem does not take ownership.
    ceres::HuberLoss loss_function_;

    template<template<typename> typename TrajectoryModel>
    friend class taser::TrajectoryEstimator;
  };

} // namespace measurements
} // namespace taser

#endif //TASERV2_STATIC_RSCAMERA_MEASUREMENT_H
