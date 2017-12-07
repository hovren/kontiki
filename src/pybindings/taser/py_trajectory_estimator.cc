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

namespace TT = taser::trajectories;
namespace TM = taser::measurements;

template<
    template<typename> typename TrajectoryModel>
auto declare_estimator(py::module &m, const TrajectoryModel<double>& dummy_DO_NOT_USE) {
  using TrajectoryImpl = TrajectoryModel<double>;
  using Class = taser::TrajectoryEstimator<TrajectoryModel>;
  namespace TM = taser::measurements;
  namespace TC = taser::cameras;

  std::string pyclass_name = "TrajectoryEstimator_" + std::string(TrajectoryImpl::CLASS_ID);

  auto cls = py::class_<Class>(m, pyclass_name.c_str());

  cls.def(py::init<std::shared_ptr<TrajectoryImpl>>());
  cls.def_property_readonly("trajectory", &Class::trajectory, "Get the trajectory");
  cls.def("solve", &Class::Solve);

  // Add all known measurement types
  hana::for_each(measurement_types, [&](auto t) {
    using MType = typename decltype(t)::type;
    cls.def("add_measurement", (void (Class::*)(std::shared_ptr<MType>)) &Class::AddMeasurement);
  });

  return cls;
}

PYBIND11_MODULE(_trajectory_estimator, m) {
  m.doc() = "Trajectory estimation class";

  // Need the ceres types since solve() returns a ceres::Solver::Summary
  py::module::import("taser._ceres");

  /* We don't seem to need these here
  py::module::import("taser.trajectories._constant_trajectory");
  py::module::import("taser.measurements._position_measurement");
  py::module::import("taser.measurements._static_rscamera_measurement");
  */

  // Create estimator for each trajectory type
  hana::for_each(trajectory_types, [&](auto t) {
    // The following unpacks the trajectory (template) and creates the double-specialization for it
    // An instance of the double-specialization is then used to give type information to
    // the declare_estimator function.
    // Not pretty, but can't find any other way since hana::template_ does not play nice
    // with template-of-templates.
    // A downside is that we now require that all trajectories have a default constructor.
    using T = typename decltype(t)::type;
    auto traj_template_t = T(); // template_t
    auto traj_double_t = traj_template_t(hana::type_c<double>);
    using TrajectoryModelDouble = typename decltype(traj_double_t)::type;

    declare_estimator(m, TrajectoryModelDouble());
  });

}