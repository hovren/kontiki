//
// Created by hannes on 2017-12-09.
//

#ifndef TASERV2_MEASUREMENT_HELPER_H
#define TASERV2_MEASUREMENT_HELPER_H

#include "../type_helpers.h"
#include <boost/hana.hpp>

template<typename Class, typename PyClass>
static void declare_measurement_common(PyClass &cls) {
  // Bind the error function for all trajectories
  hana::for_each(trajectory_types, [&](auto t) {
    using TrajectoryModel = typename decltype(t)::type;

    cls.def("error", [](Class &self, const TrajectoryModel& trajectory){
      const auto view = trajectory.AsView();
      return self.template Error<double, TrajectoryModel>(view);
    });

    cls.def("measure", [](Class &self, const TrajectoryModel& trajectory){
      const auto view = trajectory.AsView();
      return self.template Measure<double, TrajectoryModel>(view);
    });
  });
};

#endif //TASERV2_MEASUREMENT_HELPER_H
