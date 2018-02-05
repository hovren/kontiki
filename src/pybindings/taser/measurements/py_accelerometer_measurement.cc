#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include <Eigen/Dense>

#include <boost/hana.hpp>
namespace hana = boost::hana;

#include "measurement_helper.h"
#include "measurements/accelerometer_measurement.h"
#include "../imu_defs.h"

namespace py = pybind11;

PYBIND11_MODULE(_accelerometer_measurement, m) {
  m.doc() = "Accelerometer measurement";

  hana::for_each(imu_types, [&](auto imu_type) {
    using ImuModel = typename decltype(imu_type)::type;

    using Class = taser::measurements::AccelerometerMeasurement<ImuModel>;
    std::string pyclass_name = "AccelerometerMeasurement_" + std::string(ImuModel::CLASS_ID);
    auto cls = py::class_<Class, std::shared_ptr<Class>>(m, pyclass_name.c_str());
    cls.def(py::init<std::shared_ptr<ImuModel>, double, const Eigen::Vector3d &, double>());
    cls.def(py::init<std::shared_ptr<ImuModel>, double, const Eigen::Vector3d &>());

    declare_measurement_common<Class>(cls);
  }); // for_each(imu_types)

}