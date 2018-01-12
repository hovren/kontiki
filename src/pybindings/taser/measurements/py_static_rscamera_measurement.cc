#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include <Eigen/Dense>
#include <boost/hana/for_each.hpp>

#include "measurements/static_rscamera_measurement.h"
#include "sfm/landmark.h"
#include "sfm/observation.h"

#include "../type_helpers.h"
#include "measurement_helper.h"

namespace py = pybind11;

// Binds the Project() function for a TrajectoryModel
// We use a temporary dummy instance to extract the trajectory template (TrajectoryModel)
template<typename Class, typename TrajectoryModel, typename PyClass>
static void declare_project(PyClass &cls) {
  // Since multiple versions of the Project function exists, we bind to a lambda that calls the right one
  cls.def("project", [](Class &self, const TrajectoryModel& trajectory) {
    return self.template Project<double, TrajectoryModel>(trajectory.AsView());
  });
};


PYBIND11_MODULE(_static_rscamera_measurement, m) {
  m.doc() = "Static rolling shutter projection";

  // Create one StaticRsMeasurement class for each camera type
  hana::for_each(camera_types, [&](auto ct) {
    using CameraModel = typename decltype(ct)::type;

    using Class = TM::StaticRsCameraMeasurement<CameraModel>;
    std::string pyclass_name = "StaticRsCameraMeasurement_" + std::string(CameraModel::CLASS_ID);
    auto cls = py::class_<Class, std::shared_ptr<Class>>(m, pyclass_name.c_str());

    declare_measurement_common<Class>(cls);
    cls.def(py::init<std::shared_ptr<CameraModel>, std::shared_ptr<taser::Observation>>());

    cls.def_readonly("camera", &Class::camera);
    cls.def_readonly("observation", &Class::observation);

    // Declare the project() function for all trajectory types
    hana::for_each(trajectory_types, [&](auto tt) {
      using TrajectoryImpl = typename decltype(tt)::type;

      // Use temporary to extract TrajectoryModel from TrajectoryImpl
      declare_project<Class, TrajectoryImpl>(cls);

    }); // for_each(trajectory_types)
  }); // for_each(camera_types)
}