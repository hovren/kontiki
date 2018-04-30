#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include <Eigen/Dense>

#include <ceres/ceres.h>


int add(int i, int j) {
  return i + j;
}

Eigen::Vector3d vector_add(Eigen::Vector3d& a, Eigen::Vector3d& b) {
  return a + b;
}

namespace py = pybind11;

class Foo {
 public:
  int bar(int x) {
    return x * 2;
  }

  int bar(float x) {
    return x * 3;
  }
};

PYBIND11_MODULE(dummy, m) {
m.doc() = R"pbdoc(
        Pybind11 example plugin
        -----------------------

        .. currentmodule:: python_example

        .. autosummary::
           :toctree: _generate

           add
           subtract
    )pbdoc";

py::options options;

auto cls = py::class_<Foo>(m, "Foo");
  options.disable_function_signatures();
  options.disable_user_defined_docstrings();
cls.def("bar", (int (Foo::*)(int)) &Foo::bar, "Do bar");
  options.enable_user_defined_docstrings();
cls.def("bar", (int (Foo::*)(float)) &Foo::bar, "Do bar");
  options.enable_function_signatures();

m.def("add", &add, R"pbdoc(
        Add two numbers

        Some other explanation about the add function.
    )pbdoc");


m.def("subtract", [](int i, int j) { return i - j; }, R"pbdoc(
        Subtract two numbers

        Some other explanation about the subtract function.
    )pbdoc");

m.def("vector_add", &vector_add);

#ifdef VERSION_INFO
m.attr("__version__") = VERSION_INFO;
#else
m.attr("__version__") = "dev";
#endif
}