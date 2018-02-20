//
// Created by hannes on 2017-12-09.
//

#ifndef KONTIKIV2_MEASUREMENT_HELPER_H
#define KONTIKIV2_MEASUREMENT_HELPER_H

#include <boost/hana.hpp>
namespace hana = boost::hana;

#include "../trajectory_defs.h"

template<typename Class, typename PyClass>
static void declare_measurement_common(PyClass &cls) {
  // Bind the error function for all trajectories
  hana::for_each(trajectory_types, [&](auto t) {
    using TrajectoryModel = typename decltype(t)::type;

    cls.def("error", [](Class &self, const TrajectoryModel& trajectory) {
      return self.template Error<TrajectoryModel>(trajectory);
    });

    cls.def("measure", [](Class &self, const TrajectoryModel& trajectory){
      return self.template Measure<TrajectoryModel>(trajectory);
    });
  });
};

#endif //KONTIKIV2_MEASUREMENT_HELPER_H
