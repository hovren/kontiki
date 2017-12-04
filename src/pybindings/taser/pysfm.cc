//
// Created by hannes on 2017-03-20.
//

#include <string>
#include <sstream>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
namespace py = pybind11;

PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

#include "sfm/landmark.h"
#include "sfm/observation.h"
#include "sfm/view.h"

using namespace taser;

static void declare_landmark(py::module &m) {
  using Class = Landmark;
  auto cls = py::class_<Landmark, std::shared_ptr<Landmark>>(m, "Landmark");
  cls.def(py::init());
  cls.def_property_readonly("id", &Class::id, "Landmark ID");
  //cls.def("remove_observation", &Landmark::remove_observation, "Remove observation");
  //cls.def("set_reference", &Landmark::set_reference, "Set reference observation by index");
  cls.def_property("reference", &Landmark::reference, &Landmark::set_reference, "Reference observation");
  cls.def_property_readonly("observations", &Landmark::observations, "Observations");
  cls.def_property("inverse_depth", &Landmark::inverse_depth, &Landmark::set_inverse_depth, "Inverse depth");
  cls.def("__repr__", [](Landmark& self) {
    std::stringstream ss;
    ss << "<Landmark num_obs=" << self.observations().size() <<
       ", inverse depth=" << self.inverse_depth() << ">";
    return ss.str();
  });
}

static void declare_observation(py::module &m) {
  using Class = Observation;
  auto cls = py::class_<Observation, std::shared_ptr<Observation>>(m, "Observation");
  //cls.def(py::init<double, double, double, std::shared_ptr<Landmark>>());
  cls.def_property_readonly("landmark", &Observation::landmark, "Landmark");
  cls.def_property_readonly("view", &Class::view, "View");
  //cls.def_property("t0", &Observation::t0, &Observation::set_t0, "Observation frame start time");
  cls.def_property("uv", &Observation::uv, &Observation::set_uv, "Image measurement");
  cls.def("__repr__", [](Observation& self) {
    std::stringstream ss;
    ss << "<Observation lm=" << self.landmark()->id() << " f=" << self.view()->frame_nr()
       << " t0="<<self.view()->t0()
       << " uv=["<< self.uv().transpose() << "]>";
    return ss.str();
  });
}

static void declare_view(py::module &m) {
  using Class = View;
  auto cls = py::class_<Class, std::shared_ptr<Class>>(m, "View");
  cls.def(py::init<size_t, double>());
  cls.def_property("t0", &Class::t0, &Class::set_t0, "Start of frame time");
  cls.def_property("frame_nr", &Class::frame_nr, &Class::set_frame_nr, "Frame number");
  cls.def_property_readonly("observations", &Class::observations, "Observations");
  cls.def("create_observation", &Class::create_observation, "Create new observation");
  cls.def("remove_observation", &Class::remove_observation, "Remove observation");

  cls.def("__repr__", [](Class& self) {
    std::stringstream ss;
    ss << "<View frame_nr=" << self.frame_nr() << " t0="<< self.t0() << ">";
    return ss.str();
  });

  cls.def("__len__", [](Class& self){
    return self.observations().size();
  });
}

PYBIND11_MODULE(sfm, m) {
  m.doc() = "Structure from motion primitives";

  declare_observation(m);
  declare_landmark(m);
  declare_view(m);
}