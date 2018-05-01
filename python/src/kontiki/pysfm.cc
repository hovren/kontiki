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

#include <kontiki/sfm/sfm.h>

using namespace kontiki::sfm;

static void declare_landmark(py::module &m) {
  using Class = Landmark;
  auto cls = py::class_<Landmark, std::shared_ptr<Landmark>>(m, "Landmark");

  cls.doc() = R"pbdoc(A 3D landmark

  Landmarks are represented by a reference observation, and an inverse depth.
  )pbdoc";

  cls.def(py::init());
  cls.def_property_readonly("id", &Class::id,
                            R"pbdoc(Landmark ID

.. warning:: The landmark ID is only unique within a session. You can not trust an ID to be the same e.g. if loading
             the same problem from disk twice.
)pbdoc");
  cls.def_property("reference", &Landmark::reference, &Landmark::set_reference,
                   "Reference observation");
  cls.def_property_readonly("observations", &Landmark::observations,
                            "List of all landmark observations");
  cls.def_property("inverse_depth", &Landmark::inverse_depth, &Landmark::set_inverse_depth,
                   "The inverse depth, relative to the reference observation");
  cls.def_property("locked", &Class::IsLocked, &Class::Lock,
                   "Lock landmark to not allow it to change during optimization. Defaults to False.");
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

  cls.doc() = R"pbdoc(An observation of a landmark in a view
  )pbdoc";

  cls.def_property_readonly("landmark", &Observation::landmark,
                            "The :class:`Landmark` this observation belongs to");
  cls.def_property_readonly("view", &Class::view,
                            "The :class:`View` this observation belongs to");
  cls.def_property("uv", &Observation::uv, &Observation::set_uv,
                   "The image measurement.");
  cls.def_property_readonly("is_reference", &Class::IsReference,
                            "True if this is the landmark reference observation");
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
  R"pbdoc(A view (camera frame/image)
  )pbdoc";
  cls.def(py::init<size_t, double>());
  cls.def_property("t0", &Class::t0, &Class::set_t0,
                   "Start of frame time");
  cls.def_property("frame_nr", &Class::frame_nr, &Class::set_frame_nr,
                   "Frame number");
  cls.def_property_readonly("observations", &Class::observations,
                            "List of observations");
  cls.def("create_observation", &Class::CreateObservation,
          "Create a new observation");
  cls.def("remove_observation", &Class::RemoveObservation,
          "Remove an observation");

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