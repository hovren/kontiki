//
// Created by hannes on 2017-04-01.
//

#ifndef TASER_OBSERVATION_H
#define TASER_OBSERVATION_H

namespace taser {
namespace sfm {
class Landmark; // Forward declaration
class View; // Forward declaration

class Observation {
 public:
  Observation(const Eigen::Vector2d &uv, std::shared_ptr<Landmark> landmark, std::shared_ptr<View> view);

  std::shared_ptr<Landmark> landmark() const;

  std::shared_ptr<View> view() const;

  bool IsReference() const;

  Eigen::Vector2d uv() const;

  void set_uv(const Eigen::Vector2d& uv);

  double u() const;

  double v() const;

 protected:
  Eigen::Vector2d uv_;
  std::shared_ptr<Landmark> landmark_;
  std::weak_ptr<View> view_;
};

} // namespace sfm
} // namespace taser

#endif //TASER_OBSERVATION_H
