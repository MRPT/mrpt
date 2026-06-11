/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

// pybind11
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <map>
#include <string>

// MRPT headers
#include <mrpt/expr/CRuntimeCompiledExpression.h>

namespace py = pybind11;
using namespace mrpt::expr;
using namespace pybind11::literals;

PYBIND11_MODULE(_bindings, m)
{
  m.doc() = "Python bindings for mrpt-expr (Runtime expression parser)";

  py::class_<CRuntimeCompiledExpression>(m, "CRuntimeCompiledExpression", py::dynamic_attr())
      .def(py::init<>())
      // Main compilation method
      .def(
          "compile",
          [](py::object self_obj, const std::string& expression,
             const std::map<std::string, double>& variables)
          {
            auto& self = self_obj.cast<CRuntimeCompiledExpression&>();
            // exprtk stores references into `variables`, but the map built
            // by pybind11 from the Python dict is a temporary: keep a copy
            // alive for as long as the Python object lives.
            auto* vars = new std::map<std::string, double>(variables);
            self_obj.attr("_vars_keepalive") = py::capsule(
                vars, [](void* p) { delete static_cast<std::map<std::string, double>*>(p); });
            self.compile(expression, *vars);
          },
          "expression"_a, "variables"_a = std::map<std::string, double>{},
          "Compiles a string expression with optional variable and constant maps.")

      .def(
          "eval", &CRuntimeCompiledExpression::eval,
          "Evaluates the compiled expression and returns the result.")

      .def(
          "is_compiled", &CRuntimeCompiledExpression::is_compiled,
          "Returns true if the expression has been successfully compiled.")

      .def(
          "get_original_expression", &CRuntimeCompiledExpression::get_original_expression,
          "Returns the original formula string.")

      // Overloads for registering Python functions to be used inside the expression
      .def(
          "register_function",
          static_cast<void (CRuntimeCompiledExpression::*)(
              const std::string&, const std::function<double()>&)>(
              &CRuntimeCompiledExpression::register_function),
          "name"_a, "func"_a, "Registers a 0-argument Python function.")

      .def(
          "register_function",
          static_cast<void (CRuntimeCompiledExpression::*)(
              const std::string&, const std::function<double(double)>&)>(
              &CRuntimeCompiledExpression::register_function),
          "name"_a, "func"_a, "Registers a 1-argument Python function.")

      .def(
          "register_function",
          static_cast<void (CRuntimeCompiledExpression::*)(
              const std::string&, const std::function<double(double, double)>&)>(
              &CRuntimeCompiledExpression::register_function),
          "name"_a, "func"_a, "Registers a 2-argument Python function.")

      .def(
          "register_function",
          static_cast<void (CRuntimeCompiledExpression::*)(
              const std::string&, const std::function<double(double, double, double)>&)>(
              &CRuntimeCompiledExpression::register_function),
          "name"_a, "func"_a, "Registers a 3-argument Python function.");
}