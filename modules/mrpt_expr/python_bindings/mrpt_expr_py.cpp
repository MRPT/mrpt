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

  py::class_<CRuntimeCompiledExpression>(
      m, "CRuntimeCompiledExpression",
      "A wrapper of exprtk runtime expression compiler: it takes a string representing an "
      "expression (from a simple mathematical formula to a complete program), compiles it and "
      "evaluates its result as many times as required.",
      py::dynamic_attr())
      .def(py::init<>(), "Default constructor: call compile() before evaluating.")
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

      .def(
          "register_function",
          [](CRuntimeCompiledExpression& self, const std::string& name, const py::function& func)
          {
            // Any Python callable would match any std::function signature,
            // so pick the overload from the number of parameters:
            const auto nArgs =
                py::len(py::module_::import("inspect").attr("signature")(func).attr("parameters"));
            switch (nArgs)
            {
              case 0:
                self.register_function(name, func.cast<std::function<double()>>());
                break;
              case 1:
                self.register_function(name, func.cast<std::function<double(double)>>());
                break;
              case 2:
                self.register_function(name, func.cast<std::function<double(double, double)>>());
                break;
              case 3:
                self.register_function(
                    name, func.cast<std::function<double(double, double, double)>>());
                break;
              default:
                throw py::value_error(
                    "register_function(): only functions with 0 to 3 parameters are supported");
            }
          },
          "name"_a, "func"_a,
          "Registers a Python function taking 0 to 3 float arguments, for use in expressions "
          "compiled afterwards. The number of arguments is taken from its signature.");
}