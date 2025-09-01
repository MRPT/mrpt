/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

// pybind11
#include <pybind11/chrono.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

// MRPT headers
#include <mrpt/containers/yaml.h>

#include <fstream>
#include <sstream>
#include <string>

namespace py = pybind11;

PYBIND11_MODULE(_bindings, m)
{
  using mrpt::containers::yaml;

  m.doc() = "Python bindings for mrpt_containers";

  py::class_<yaml>(m, "YAML")
      .def(py::init<>())  // default constructor

      // --- Static constructors ---
      .def_static("from_string", &yaml::FromText, "Parse YAML from string")
      .def_static("from_file", &yaml::FromFile, "Parse YAML from file")

      // --- Serialization ---
      .def(
          "to_string",
          [](const yaml &obj)
          {
            std::stringstream ss;
            obj.printAsYAML(ss);
            return ss.str();
          },
          "Dump YAML to string")
#if 0
      .def(
          "save_to_file",
          [](const yaml &o, const std::string &fileName)
          {
            std::ofstream f(fileName);
            if (!f.is_open())
            {
              return;
            }
            o.printAsYAML(f);
          },
          py::arg("fileName"), py::arg("opts") = mrpt::containers::YamlEmitOptions{},
          "Save YAML to a file")
#endif
      // --- Basic API ---
      .def("has", &yaml::has, "Check if a key exists")
      .def("clear", &yaml::clear, "Clear all contents")
      .def("size", &yaml::size, "Number of elements")
      .def("empty", &yaml::empty, "Check if YAML is empty")

  // --- Operators mapped to Python ---
#if 0
      .def("__getitem__", [](const yaml &y, const std::string &key) { return y.at(key); })
#endif
      .def("__setitem__", [](yaml &y, const std::string &key, const yaml &val) { y[key] = val; })
      .def(
          "__repr__",
          [](const yaml &y)
          {
            std::stringstream ss;
            y.printAsYAML(ss);
            return ss.str();
          });
}
