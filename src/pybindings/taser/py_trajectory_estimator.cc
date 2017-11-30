#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include <Eigen/Dense>

#include "trajectory_estimator.h"
#include "trajectories/linear_trajectory.h"
#include "trajectories/constant_trajectory.h"
#include "measurements/position_measurement.h"

namespace py = pybind11;

// Recursion base case. Ignore this.
template<typename Class, typename PyClass>
void declare_add_measurement(PyClass& cls) {
  // This shall do nothing
};

// Bind AddMeasurement<T> for all listed measurement types T
template<typename Class, typename PyClass, typename MType, typename... MTargs>
void declare_add_measurement(PyClass& cls) {
  // Add first type
  cls.def("add_measurement", (void (Class::*)(std::shared_ptr<MType>)) &Class::AddMeasurement);

  // Recurse through the rest of the measurement types
  declare_add_measurement<Class, PyClass, MTargs...>(cls);
};

template<
    template<typename> typename TrajectoryModel>
auto declare_estimator(py::module &m) {
  using TrajectoryImpl = TrajectoryModel<double>;
  using Class = taser::TrajectoryEstimator<TrajectoryModel>;
  namespace TM = taser::measurements;

  std::string pyclass_name = "TrajectoryEstimator_" + std::string(TrajectoryImpl::CLASS_ID);

  auto cls = py::class_<Class>(m, pyclass_name.c_str());

  cls.def(py::init<std::shared_ptr<TrajectoryImpl>>());
  cls.def_property_readonly("trajectory", &Class::trajectory, "Get the trajectory");
  cls.def("solve", &Class::Solve);

  // Add list of measurement types to this estimator type
  declare_add_measurement<Class, typeof(cls),
                          /* List of measurement types starts here */
                          TM::PositionMeasurement,
                          TM::AnotherMeasurement>(cls);

  return cls;
}

PYBIND11_MODULE(_trajectory_estimator, m) {
  m.doc() = "Trajectory estimation class";

  py::module::import("taser._ceres");
  py::module::import("taser.trajectories._constant_trajectory");
  py::module::import("taser.measurements._position_measurement");

  namespace TT = taser::trajectories;
  namespace TM = taser::measurements;
  declare_estimator<TT::ConstantTrajectory>(m);
  declare_estimator<TT::LinearTrajectory>(m);
  //declare_estimator<TT::ConstantTrajectory, TM::PositionMeasurement>(m);
}