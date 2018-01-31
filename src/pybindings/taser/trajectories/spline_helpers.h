#ifndef TASERV2_SPLINE_HELPERS_H
#define TASERV2_SPLINE_HELPERS_H

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

namespace py = pybind11;

// Translate a Python sequence index i (with negative for from the end indexing) to a linear index,
// given sequence length N
constexpr int python_index_to_linear(int i, int N) {
  if ((i >= N) || (i < -N)) {
    throw py::index_error("Invalid sequence index");
  }
  else if ( i < 0 )
    return i + N;
  else
    return i;
}

// Declare common attributes for splined trajectories
// Since pybind11 can't handle e.g. the Eigen::Quaternion type we use functions
// within the Helper class to convert between the Python and C++ control point types
template<typename Class, typename Helper, typename PyClass>
void declare_spline_common(PyClass& cls) {
  using PyCPType = typename Helper::PyControlPointType;
  cls.def(py::init<>());
  cls.def(py::init<double>());
  cls.def(py::init<double, double>());
  cls.def_property_readonly("dt", &Class::dt);
  cls.def_property_readonly("t0", &Class::t0);
  cls.def("__len__", &Class::NumKnots);
  cls.def("__getitem__", [&](Class& self, int i) {
    PyCPType cp = Helper::ConvertCppToPy(
        self.ControlPoint(python_index_to_linear(i, self.NumKnots())));
    return cp;
  });
  cls.def("__setitem__", [&](Class& self, int i, PyCPType& cp) {
    self.MutableControlPoint(python_index_to_linear(i, self.NumKnots())) = Helper::ConvertPyToCpp(cp);
  });
  cls.def("append_knot", [&](Class& self, PyCPType& cp) {
    return self.AppendKnot(Helper::ConvertPyToCpp(cp));
  });
//  cls.def("extend_to", [&](Class& self, double t, const PyCPType& fill_value) {
//    return self.ExtendTo(t, Helper::ConvertPyToCpp(fill_value));
//  });
};

#endif //TASERV2_SPLINE_HELPERS_H
