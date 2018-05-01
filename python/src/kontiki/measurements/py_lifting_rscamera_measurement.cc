#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include <Eigen/Dense>
#include <boost/hana.hpp>
namespace hana = boost::hana;

#include <kontiki/types.h>
#include <kontiki/measurements/lifting_rscamera_measurement.h>
#include <kontiki/sfm/landmark.h>
#include <kontiki/sfm/observation.h>

#include "../camera_defs.h"
#include "../trajectory_defs.h"

#include "measurement_helper.h"


namespace py = pybind11;

PYBIND11_MODULE(_lifting_rscamera_measurement, m) {
  m.doc() = "Lifting rolling shutter projection";

  // Create one StaticRsMeasurement class for each camera type
  hana::for_each(camera_types, [&](auto ct) {
    using CameraModel = typename decltype(ct)::type;

    using Class = kontiki::measurements::LiftingRsCameraMeasurement<CameraModel>;
    std::string pyclass_name = "LiftingRsCameraMeasurement_" + std::string(CameraModel::CLASS_ID);
    auto cls = py::class_<Class, std::shared_ptr<Class>>(m, pyclass_name.c_str());

    cls.doc() = R"pbdoc( Rolling shutter projection by time optimization

    Projects a landmark into a rolling shutter camera by adding the projection time
    as an additional parameter to the optimization problem.

    In general, this does not fulfill the rolling shutter projection time constraint exactly.
    )pbdoc";

    declare_measurement_common<Class>(cls);
    cls.def(py::init<std::shared_ptr<CameraModel>, std::shared_ptr<kontiki::sfm::Observation>, double, double>());
    cls.def(py::init<std::shared_ptr<CameraModel>, std::shared_ptr<kontiki::sfm::Observation>, double>());
    cls.def(py::init<std::shared_ptr<CameraModel>, std::shared_ptr<kontiki::sfm::Observation>>());

    cls.def_readonly("camera", &Class::camera);
    cls.def_readonly("observation", &Class::observation);
    cls.def_readwrite("weight", &Class::weight, "Image residual weight (applied before loss)");
    cls.def_readonly("vt", &Class::vt_, "Row projection time (to be optimized)");

    // Declare the project() function for all trajectory types
    hana::for_each(trajectory_types, [&](auto tt) {
      using TrajectoryModel = typename decltype(tt)::type;

      // Use temporary to extract TrajectoryModel from TrajectoryImpl
      cls.def("project", [](Class &self, const TrajectoryModel &trajectory) {
        return self.template Project<TrajectoryModel>(trajectory);
        });

    }); // for_each(trajectory_types)
  }); // for_each(camera_types)
}