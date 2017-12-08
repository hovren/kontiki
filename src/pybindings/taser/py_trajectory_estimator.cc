#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include <Eigen/Dense>
#include <cameras/atan.h>

#include "trajectory_estimator.h"
#include "trajectories/linear_trajectory.h"
#include "trajectories/constant_trajectory.h"
#include "measurements/position_measurement.h"
#include "measurements/static_rscamera_measurement.h"
#include "cameras/pinhole.h"

#include "type_helpers.h"

#include <boost/hana.hpp>
namespace hana = boost::hana;

namespace py = pybind11;

PYBIND11_MODULE(_trajectory_estimator, m) {
  m.doc() = "Trajectory estimation class";

  // Need the ceres types since solve() returns a ceres::Solver::Summary
  py::module::import("taser._ceres");

  // Create estimator for each trajectory type
  hana::for_each(trajectory_types, [&](auto t) {
    // Unpack trajectory type
    using T = typename decltype(t)::type;
    auto traj_template = T();
    auto traj_double = traj_template(hana::type_c<double>);
    using TrajectoryImpl = typename decltype(traj_double)::type;

    // Unpack estimator type
    auto estimator_t = template_template<taser::TrajectoryEstimator>(t);
    using Class = typename decltype(estimator_t)::type;

    // Begin python class binding for TrajectoryEstimator<TrajectoryModel>
    std::string pyclass_name = "TrajectoryEstimator_" + std::string(TrajectoryImpl::CLASS_ID);
    auto cls = py::class_<Class>(m, pyclass_name.c_str());

    cls.def(py::init<std::shared_ptr<TrajectoryImpl>>());
    cls.def_property_readonly("trajectory", &Class::trajectory, "Get the trajectory");
    cls.def("solve", &Class::Solve);

    // Add all known measurement types
    hana::for_each(measurement_types, [&](auto tm) {
      // Unpack measurement type
      using MType = typename decltype(tm)::type;

      // Unpack estimator type again, because of some reason
      auto estimator_t = template_template<taser::TrajectoryEstimator>(t);
      using Class = typename decltype(estimator_t)::type;

      cls.def("add_measurement", (void (Class::*)(std::shared_ptr<MType>)) &Class::AddMeasurement);
    });
  });

}