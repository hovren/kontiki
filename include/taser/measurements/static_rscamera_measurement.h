//
// Created by hannes on 2017-12-04.
//

#ifndef TASERV2_STATIC_RSCAMERA_MEASUREMENT_H
#define TASERV2_STATIC_RSCAMERA_MEASUREMENT_H

#include "trajectories/trajectory.h"
#include "sfm/observation.h"
#include "sfm/landmark.h"
#include "sfm/view.h"

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

  // FIXME: Relative rotation! (Moved to camera?)
  Vector3 p_ct = Vector3::Zero();
  Eigen::Quaternion<T> q_ct = Eigen::Quaternion<T>::Identity();

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
   public:
    StaticRsCameraMeasurement(std::shared_ptr<CameraImpl> camera, std::shared_ptr<Landmark> landmark, std::shared_ptr<Observation> obs)
        : camera(camera), landmark(landmark), observation(obs) {};

    std::shared_ptr<CameraImpl> camera;
    // Measurement data
    std::shared_ptr<taser::Landmark> landmark;
    std::shared_ptr<taser::Observation> observation;

    template<typename T, template<typename> typename TrajectoryModel>
    Eigen::Matrix<T, 2, 1> Project(const TrajectoryModel<T> &trajectory) const {
      return reproject_static(*landmark->reference(), *observation, landmark->inverse_depth(), trajectory, *camera);
    };

    template<typename T, template<typename> typename TrajectoryModel>
    Eigen::Matrix<T, 2, 1> error(const TrajectoryModel<T> &trajectory) const {
      Eigen::Matrix<T,2,1> y_hat = this->Project(trajectory);
      return observation->uv().cast<T>() - y_hat;
    }

  };
} // namespace measurements
} // namespace taser

#endif //TASERV2_STATIC_RSCAMERA_MEASUREMENT_H
