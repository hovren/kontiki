#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include "trajectory_estimator.h"

#include <boost/hana.hpp>
namespace hana = boost::hana;

#include "trajectory_defs.h"
#include "measurement_defs.h"

namespace py = pybind11;

PYBIND11_MODULE(_trajectory_estimator, m) {
  m.doc() = "Trajectory estimation class";

  // Need the ceres types since solve() returns a ceres::Solver::Summary
  py::module::import("taser._ceres");

  // Create estimator for each trajectory type
  hana::for_each(trajectory_types, [&](auto t) {
    // Unpack trajectory type
    using TrajectoryImpl = typename decltype(t)::type;

    // Define estimator type
    using Class = taser::TrajectoryEstimator<TrajectoryImpl>;

    // Begin python class binding for TrajectoryEstimator<TrajectoryModel>
    std::string pyclass_name = "TrajectoryEstimator_" + std::string(TrajectoryImpl::CLASS_ID);
    auto cls = py::class_<Class>(m, pyclass_name.c_str());

    cls.def(py::init<std::shared_ptr<TrajectoryImpl>>());
    cls.def_property_readonly("trajectory", &Class::trajectory, "Get the trajectory");
    cls.def("solve", &Class::Solve, "Solve the current estimation problem",
            py::arg("max_iterations")=50, py::arg("progress")=true, py::arg("num_threads")=-1);

    // Add all known measurement types
    hana::for_each(measurement_types, [&](auto tm) {
      // Unpack measurement type
      using MType = typename decltype(tm)::type;

      // Unpack estimator type again, because of some reason
      using Class = taser::TrajectoryEstimator<TrajectoryImpl>;

      cls.def("add_measurement", (void (Class::*)(std::shared_ptr<MType>)) &Class::AddMeasurement,
        py::keep_alive<1, 2>()); // Keep measurement (2) alive as long as the estimator (1)
    });
  });

}