//
// Created by hannes on 2017-03-21.
//
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
namespace py = pybind11;

#include <ceres/ceres.h>

void declare_summary(py::module& m) {
  typedef ceres::Solver::Summary Class;
  py::class_ <ceres::Solver> solver(m, "Solver");

  py::class_<Class>(m, "Summary", solver)
      .def(py::init())
      .def("BriefReport", &Class::BriefReport)
      .def("FullReport", &Class::FullReport)
      .def("IsSolutionUsable", &Class::IsSolutionUsable)
      .def_readonly("message", &Class::message)
      .def_readonly("initial_cost", &Class::initial_cost)
      .def_readonly("iterations", &Class::iterations)
      .def_readonly("final_cost", &Class::final_cost)
      .def_readonly("fixed_cost", &Class::fixed_cost)
      .def_readonly("termination_type", &Class::termination_type)
      .def_readonly("num_successful_steps", &Class::num_successful_steps)
      .def_readonly("num_unsuccessful_steps", &Class::num_unsuccessful_steps)
      .def_readonly("num_inner_iteration_steps", &Class::num_inner_iteration_steps)
      .def_readonly("preprocessor_time_in_seconds", &Class::preprocessor_time_in_seconds)
      .def_readonly("minimizer_time_in_seconds", &Class::minimizer_time_in_seconds)
      .def_readonly("postprocessor_time_in_seconds", &Class::postprocessor_time_in_seconds)
      .def_readonly("total_time_in_seconds", &Class::total_time_in_seconds)
      .def_readonly("linear_solver_time_in_seconds", &Class::linear_solver_time_in_seconds)
      .def_readonly("residual_evaluation_time_in_seconds", &Class::residual_evaluation_time_in_seconds)
      .def_readonly("jacobian_evaluation_time_in_seconds", &Class::jacobian_evaluation_time_in_seconds)
      .def_readonly("inner_iteration_time_in_seconds", &Class::inner_iteration_time_in_seconds)
      .def_readonly("line_search_cost_evaluation_time_in_seconds",
                    &Class::line_search_cost_evaluation_time_in_seconds)
      .def_readonly("line_search_gradient_evaluation_time_in_seconds",
                    &Class::line_search_gradient_evaluation_time_in_seconds)
      .def_readonly("line_search_polynomial_minimization_time_in_seconds",
                    &Class::line_search_polynomial_minimization_time_in_seconds)
      .def_readonly("line_search_total_time_in_seconds", &Class::line_search_total_time_in_seconds)
      .def_readonly("num_parameter_blocks", &Class::num_parameter_blocks)
      .def_readonly("num_parameters", &Class::num_parameters)
      .def_readonly("num_effective_parameters", &Class::num_effective_parameters)
      .def_readonly("num_residual_blocks", &Class::num_residual_blocks)
      .def_readonly("num_residuals", &Class::num_residuals)
      .def_readonly("num_parameter_blocks_reduced", &Class::num_parameter_blocks_reduced)
      .def_readonly("num_parameters_reduced", &Class::num_parameters_reduced)
      .def_readonly("num_effective_parameters_reduced", &Class::num_effective_parameters_reduced)
      .def_readonly("num_residual_blocks_reduced", &Class::num_residual_blocks_reduced)
      .def_readonly("num_residuals_reduced", &Class::num_residuals_reduced)
      .def_readonly("is_constrained", &Class::is_constrained)
      .def_readonly("num_threads_given", &Class::num_threads_given)
      .def_readonly("num_threads_used", &Class::num_threads_used)
      .def_readonly("num_linear_solver_threads_given", &Class::num_linear_solver_threads_given)
      .def_readonly("num_linear_solver_threads_used", &Class::num_linear_solver_threads_used); // end Class
}

void declare_iteration_callback(py::module& m) {
  py::enum_<ceres::CallbackReturnType> e(m, "CallbackReturnType");
  e.value("Abort", ceres::CallbackReturnType::SOLVER_ABORT);
  e.value("Continue", ceres::CallbackReturnType::SOLVER_CONTINUE);
  e.value("TerminateSuccessfully", ceres::CallbackReturnType::SOLVER_TERMINATE_SUCCESSFULLY);
}

void declare_iteration_summary(py::module& m) {
  typedef ceres::IterationSummary Class;
  py::class_<Class> cls(m, "IterationSummary");

  cls.def_readonly("iteration", &Class::iteration, "Current iteration");
  cls.def_readonly("step_is_valid", &Class::step_is_valid, "Step was numerically valid");
  cls.def_readonly("step_is_nonmonotonic", &Class::step_is_nonmonotonic);
  cls.def_readonly("step_is_successful", &Class::step_is_successful);
  cls.def_readonly("cost", &Class::cost);
  cls.def_readonly("cost_change", &Class::cost_change);
  cls.def_readonly("gradient_max_norm", &Class::gradient_max_norm);
  cls.def_readonly("gradient_norm", &Class::gradient_norm);
  cls.def_readonly("step_norm", &Class::step_norm);
  cls.def_readonly("relative_decrease", &Class::relative_decrease);
  cls.def_readonly("trust_region_radius", &Class::trust_region_radius);
  cls.def_readonly("eta", &Class::eta);
  cls.def_readonly("step_size", &Class::step_size);
  cls.def_readonly("line_search_function_evaluations", &Class::line_search_function_evaluations);
  cls.def_readonly("line_search_gradient_evaluations", &Class::line_search_gradient_evaluations);
  cls.def_readonly("line_search_iterations", &Class::line_search_iterations);
  cls.def_readonly("linear_solver_iterations", &Class::linear_solver_iterations);
  cls.def_readonly("iteration_time_in_seconds", &Class::iteration_time_in_seconds);
  cls.def_readonly("step_solver_time_in_seconds", &Class::step_solver_time_in_seconds);
  cls.def_readonly("cumulative_time_in_seconds", &Class::cumulative_time_in_seconds);
}

void declare_termination_type(py::module &m) {
  py::enum_<ceres::TerminationType> e(m, "TerminationType");
  e.value("Convergence", ceres::TerminationType::CONVERGENCE);
  e.value("NoConvergence", ceres::TerminationType::NO_CONVERGENCE);
  e.value("Failure", ceres::TerminationType::FAILURE);
  e.value("UserSuccess", ceres::TerminationType::USER_SUCCESS);
  e.value("UserFailure", ceres::TerminationType::USER_FAILURE);
}

PYBIND11_MODULE(_ceres, m) {
  m.doc() = "ceres-solver types";

  declare_iteration_summary(m);
  declare_summary(m);
  declare_termination_type(m);
  declare_iteration_callback(m);
}