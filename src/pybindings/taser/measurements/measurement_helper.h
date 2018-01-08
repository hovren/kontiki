//
// Created by hannes on 2017-12-09.
//

#ifndef TASERV2_MEASUREMENT_HELPER_H
#define TASERV2_MEASUREMENT_HELPER_H

#include "../type_helpers.h"
#include <boost/hana.hpp>

template<typename Class, typename TrajectoryImpl, typename PyClass>
static void declare_error_for_trajectory(PyClass &cls) {
  // Since multiple versions of the Error function exists, we bind to a lambda that calls the right one
  cls.def("error", [](Class &self, const TrajectoryImpl& trajectory){
    const auto view = trajectory.AsView();
    return self.template Error<double, TrajectoryImpl>(view);
  });

  cls.def("measure", [](Class &self, const TrajectoryImpl& trajectory){
    const auto view = trajectory.AsView();
    return self.template Measure<double, TrajectoryImpl>(view);
  });
};

template<typename Class, typename PyClass>
static void declare_measurement_common(PyClass &cls) {
  // Bind the error function for all trajectories
  hana::for_each(trajectory_types, [&](auto t) {
//    auto traj_double_type = t(hana::type_c<double>);
//    using TrajectoryImpl = typename decltype(traj_double_type)::type;
    using TrajectoryImpl = typename decltype(t)::type;

    // Use temporary to extract TrajectoryModel from TrajectoryImpl
    declare_error_for_trajectory<Class, TrajectoryImpl>(cls);
  });
};

#endif //TASERV2_MEASUREMENT_HELPER_H
