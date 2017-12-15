//
// Created by hannes on 2017-12-09.
//

#ifndef TASERV2_MEASUREMENT_HELPER_H
#define TASERV2_MEASUREMENT_HELPER_H

#include "../type_helpers.h"
#include <boost/hana.hpp>

template<typename Class, typename PyClass, template<typename> typename TrajectoryModel>
static void declare_error_for_trajectory(PyClass &cls, const TrajectoryModel<double>& dummy_DO_NOT_USE) {
  // Since multiple versions of the Error function exists, we bind to a lambda that calls the right one
  cls.def("error", [](Class &self, const TrajectoryModel<double>& trajectory){
    return self.Error(trajectory);
  });
};

template<typename Class, typename PyClass>
static void declare_measurement_common(PyClass &cls) {
  // Bind the error function for all trajectories
  hana::for_each(trajectory_types, [&](auto t) {
    auto traj_double_type = t(hana::type_c<double>);
    using TrajectoryImpl = typename decltype(traj_double_type)::type;

    // Use temporary to extract TrajectoryModel from TrajectoryImpl
    declare_error_for_trajectory<Class>(cls, TrajectoryImpl());
  });
};

#endif //TASERV2_MEASUREMENT_HELPER_H
