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
  Observation(double u, double v, std::shared_ptr<Landmark> landmark, std::shared_ptr<View> view) :
      u_(u), v_(v), landmark_(landmark), view_(view) {};

  auto landmark() const {
    return landmark_;
  };

  auto view() const {
    auto sp = view_.lock();
    if (sp)
      return sp;
    else
      throw std::runtime_error("View does not exist anymore");
  }

  Eigen::Vector2d uv() const { return Eigen::Vector2d(u_, v_); }
  void set_uv(const Eigen::Vector2d& uv) {
    u_ = uv(0);
    v_ = uv(1);
  }
  auto u() const { return u_; }
  auto v() const { return v_; }
 protected:
  double u_, v_;
  std::shared_ptr<Landmark> landmark_;
  std::weak_ptr<View> view_;
};

} // namespace taser

#endif //TASER_OBSERVATION_H
