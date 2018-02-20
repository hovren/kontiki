//
// Created by hannes on 2018-02-07.
//

#ifndef KONTIKIV2_IMU_HELPER_H
#define KONTIKIV2_IMU_HELPER_H

#include <boost/hana.hpp>
namespace hana = boost::hana;

#include "sensors_helper.h"
#include "../trajectory_defs.h"


template<typename Class, typename PyClass>
static void declare_imu_common(PyClass &cls) {

  declare_sensors_common<Class>(cls);

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

#endif //KONTIKIV2_IMU_HELPER_H
