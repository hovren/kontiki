//
// Created by hannes on 2017-10-04.
//

#ifndef TASER_VIEW_H
#define TASER_VIEW_H

#include <vector>
#include <memory>

#include "observation.h"
#include "landmark.h"

namespace taser {

class View : public std::enable_shared_from_this<View> {
 public:
  View(size_t frame, double t0)
      : frame_nr_(frame), t0_(t0) {  };

  ~View() {
    // Make sure a View that is destroyed propagate observations removed
    std::vector<std::shared_ptr<Observation>> obs_copy(observations_);
    for (auto obs : obs_copy) {
      remove_observation(obs);
    }
  }

  auto frame_nr() const { return frame_nr_; }
  void set_frame_nr(size_t fnr) { frame_nr_ = fnr; }
  auto t0() const { return t0_; }
  void set_t0(double t0) { t0_ = t0; }
  auto observations() const { return observations_; }

  auto create_observation(std::shared_ptr<Landmark> landmark, double u, double v) {
    auto obs = std::make_shared<Observation>(u, v, landmark, shared_from_this());
    observations_.push_back(obs);
    landmark->observations_.push_back(obs);
    return obs;
  }

  void remove_observation(std::shared_ptr<Observation> obs) {
    auto it = std::find(observations_.begin(), observations_.end(), obs);
    if (it != observations_.end()) {
      observations_.erase(it);
    }
    else {
      throw std::runtime_error("Observation does not beloing to this view");
    }

    obs->landmark()->remove_observation(obs);
  }

 protected:
  size_t frame_nr_;
  double t0_;
  std::vector<std::shared_ptr<Observation>> observations_;
};

} // namespace taser
#endif //TASER_VIEW_H
