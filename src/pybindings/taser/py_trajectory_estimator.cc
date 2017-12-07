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
auto declare_estimator(py::module &m) {
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

  py::module::import("taser._ceres");
  py::module::import("taser.trajectories._constant_trajectory");
  py::module::import("taser.measurements._position_measurement");
  py::module::import("taser.measurements._static_rscamera_measurement");

  namespace TT = taser::trajectories;
  namespace TM = taser::measurements;
  //declare_estimator<TT::ConstantTrajectory>(m);
  declare_estimator<TT::LinearTrajectory>(m);
}