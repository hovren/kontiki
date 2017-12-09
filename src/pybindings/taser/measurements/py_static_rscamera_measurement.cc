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
template<typename Class, typename PyClass,
    template<typename> typename TrajectoryModel>
static void declare_project(PyClass &cls, const TrajectoryModel<double> &dummy_DO_NOT_USE) {
  cls.def("project", &Class::template Project<double, TrajectoryModel>);
};

PYBIND11_MODULE(_static_rscamera_measurement, m) {
  m.doc() = "Static rolling shutter projection";

  // Create one StaticRsMeasurement class for each camera type
  hana::for_each(camera_types, [&](auto t) {
    using CameraModel = typename decltype(t)::type;

    using Class = TM::StaticRsCameraMeasurement<CameraModel>;
    std::string pyclass_name = "StaticRsCameraMeasurement_" + std::string(CameraModel::CLASS_ID);
    auto cls = py::class_<Class, std::shared_ptr<Class>>(m, pyclass_name.c_str());

    declare_measurement_common<Class>(cls);
    cls.def(py::init<std::shared_ptr<CameraModel>, std::shared_ptr<taser::Landmark>, std::shared_ptr<taser::Observation>>());

    // Declare the project() function for all trajectory types
    hana::for_each(trajectory_types, [&](auto t) {
      auto traj_double_type = t(hana::type_c<double>);
      using TrajectoryImpl = typename decltype(traj_double_type)::type;

      // Use temporary to extract TrajectoryModel from TrajectoryImpl
      declare_project<Class>(cls, TrajectoryImpl());
    }); // for_each(trajectory_types)
  }); // for_each(camera_types)
}