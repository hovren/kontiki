//
// Created by hannes on 2017-04-01.
//

#ifndef TASER_OBSERVATION_H
#define TASER_OBSERVATION_H

namespace taser {
class Landmark; // Forward declaration
class View; // Forward declaration

class Observation {
 public:
  Observation(const Eigen::Vector2d &uv, std::shared_ptr<Landmark> landmark, std::shared_ptr<View> view) :
      uv_(uv),
      landmark_(landmark),
      view_(view) { };

  std::shared_ptr<Landmark> landmark() const {
    return landmark_;
  };

  std::shared_ptr<View> view() const {
    auto sp = view_.lock();
    if (sp)
      return sp;
    else
      throw std::runtime_error("View does not exist anymore");
  }

  Eigen::Vector2d uv() const {
    return uv_;
  }

  void set_uv(const Eigen::Vector2d& uv) {
    uv_ = uv;
  }

  double u() const {
    return uv_(0);
  }

  double v() const {
    return uv_(1);
  }

 protected:
  Eigen::Vector2d uv_;
  std::shared_ptr<Landmark> landmark_;
  std::weak_ptr<View> view_;
};

} // namespace taser

#endif //TASER_OBSERVATION_H
