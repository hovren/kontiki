//
// Created by hannes on 2018-01-31.
//

#ifndef TASERV2_SENSORS_HELPER_H
#define TASERV2_SENSORS_HELPER_H

#include <boost/hana.hpp>

#include "../type_helpers.h"

template<typename Class, typename PyClass>
static void declare_imu_common(PyClass &cls) {
  // Bind functions that need to know the trajectory type
  hana::for_each(trajectory_types, [&](auto t) {
    using TrajectoryModel = typename decltype(t)::type;

    cls.def("accelerometer", [](Class &self, const TrajectoryModel& trajectory, double t) {
      return self.template Accelerometer<TrajectoryModel>(trajectory, t);
    });

    cls.def("gyroscope", [](Class &self, const TrajectoryModel& trajectory, double t) {
      return self.template Gyroscope<TrajectoryModel>(trajectory, t);
    });
  }); // for_each trajectory type
}


#endif //TASERV2_SENSORS_HELPER_H
