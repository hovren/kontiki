#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include <Eigen/Dense>

#include "measurements/static_rscamera_measurement.h"
#include "cameras/pinhole.h"
#include "cameras/atan.h"
#include "sfm/landmark.h"
#include "sfm/observation.h"
#include "trajectories/linear_trajectory.h"

namespace py = pybind11;
namespace TM = taser::measurements;
namespace TC = taser::cameras;
namespace TT = taser::trajectories;

template<typename Class, typename PyClass>
static void declare_project(PyClass &cls) {
  // Nothing
};

template<typename Class, typename PyClass,
    template<typename> typename TrajectoryModel,
    template<typename> typename... OtherTrajectoryModels>
static void declare_project(PyClass &cls) {
  cls.def("project", &Class::template Project<double, TrajectoryModel>);

  // Recurse through the rest of the trajectory models
  declare_project<Class, PyClass, OtherTrajectoryModels...>(cls);
};

template<typename CameraModel>
static void declare_rsstatic_measurement(py::module &m) {
  using Class = TM::StaticRsCameraMeasurement<CameraModel>;
  std::string pyclass_name = "StaticRsCameraMeasurement_" + std::string(CameraModel::CLASS_ID);
  auto cls = py::class_<Class, std::shared_ptr<Class>>(m, pyclass_name.c_str());

  cls.def(py::init<std::shared_ptr<CameraModel>, std::shared_ptr<taser::Landmark>, std::shared_ptr<taser::Observation>>());

  //cls.def("project", &Class::template Project<double, TT::LinearTrajectory>);
  declare_project<Class, typeof(cls),
      TT::LinearTrajectory
      >(cls);
}

PYBIND11_MODULE(_static_rscamera_measurement, m) {
  m.doc() = "Static rolling shutter projection";

  declare_rsstatic_measurement<TC::PinholeCamera>(m);
  declare_rsstatic_measurement<TC::AtanCamera>(m);
}