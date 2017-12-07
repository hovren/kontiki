#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include <Eigen/Dense>
#include <boost/hana/for_each.hpp>

#include "measurements/static_rscamera_measurement.h"
#include "sfm/landmark.h"
#include "sfm/observation.h"

#include "../type_helpers.h"

namespace py = pybind11;

template<typename Class, typename PyClass,
    template<typename> typename TrajectoryModel>
static void declare_project(PyClass &cls, const TrajectoryModel<double> &dummy_DO_NOT_USE) {
  cls.def("project", &Class::template Project<double, TrajectoryModel>);
};

template<typename CameraModel>
static void declare_rsstatic_measurement(py::module &m) {
  using Class = TM::StaticRsCameraMeasurement<CameraModel>;
  std::string pyclass_name = "StaticRsCameraMeasurement_" + std::string(CameraModel::CLASS_ID);
  auto cls = py::class_<Class, std::shared_ptr<Class>>(m, pyclass_name.c_str());

  cls.def(py::init<std::shared_ptr<CameraModel>, std::shared_ptr<taser::Landmark>, std::shared_ptr<taser::Observation>>());

  // Add the project() member for all trajectory types
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

    declare_project<Class>(cls, TrajectoryModelDouble());
  });
}

PYBIND11_MODULE(_static_rscamera_measurement, m) {
  m.doc() = "Static rolling shutter projection";

  // Create one StaticRsMeasurement class for each camera type
  hana::for_each(camera_types, [&](auto t) {
    using T = typename decltype(t)::type;
    declare_rsstatic_measurement<T>(m);
  });
}