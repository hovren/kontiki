#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include <Eigen/Dense>
#include <trajectory_estimator.h>

#include "trajectories/constant_trajectory.h"
#include "measurements/position_measurement.h"

namespace py = pybind11;

#if 1
template<typename Class, typename PyClass, typename MType>
void declare_add_measurement(PyClass& cls) {
#pragma message("ADD MEASUREMENT LEVEL 0")
  //cls.def("AddMeasurement", &Class::AddMeasurement<MType>);
  std::cout << "declare_add_measurement" << std::endl;
  cls.def("add_measurement", (void (Class::*)(std::shared_ptr<MType>)) &Class::AddMeasurement);
};

#elif 0
template<typename Class, typename PyClass, typename MType, typename... MTargs>
void declare_add_measurement(PyClass& cls) {
#pragma message("ADD MEASUREMENT LEVEL N")
  cls.def("AddMeasurement", &Class::AddMeasurement<MType>);
  declare_add_measurement<Class, PyClass, MTargs...>(cls);
};

#endif

class DummyMeasurement {
 public:
  DummyMeasurement() {};
};

template<
    template<typename> typename TrajectoryModel>
auto declare_estimator(py::module &m) {
  using TrajectoryImpl = TrajectoryModel<double>;
  using Class = taser::TrajectoryEstimator<TrajectoryModel>;
  namespace TM = taser::measurements;

  auto cls = py::class_<Class>(m, "TrajectoryEstimator");

  cls.def(py::init<std::shared_ptr<TrajectoryImpl>>());
  cls.def_property_readonly("trajectory", &Class::trajectory, "Get the trajectory");
  cls.def("solve", &Class::Solve);

  //declare_add_measurement<Class, typeof(cls), MTargs...>(cls);

  taser::measurements::PositionMeasurement meas(0, Eigen::Vector3d(0, 0, 0));

  /*
  cls.def("add_measurement",
          (void (Class::*)(std::shared_ptr<taser::measurements::PositionMeasurement>))
              &Class::AddMeasurement);
  */

  declare_add_measurement<Class, typeof(cls), TM::PositionMeasurement>(cls);
  declare_add_measurement<Class, typeof(cls), TM::AnotherMeasurement>(cls);

  //cls.def("test", &Class::test<double>);

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
  //declare_estimator<TT::ConstantTrajectory, TM::PositionMeasurement>(m);
}