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
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

// MRPT headers
#include <mrpt/containers/yaml.h>

#include <fstream>
#include <sstream>
#include <string>

namespace py = pybind11;
using mrpt::containers::yaml;

PYBIND11_MODULE(_bindings, m)
{
  m.doc() = "Python bindings for mrpt_containers";

  py::class_<yaml>(m, "YAML")
      .def(py::init<>())

      // --- Static constructors ---
      .def_static("from_string", &yaml::FromText, "Parse YAML/JSON from string")
      .def_static("from_file", &yaml::FromFile, "Parse YAML/JSON from file")

      // --- Serialization ---
      .def(
          "to_string",
          [](const yaml& obj)
          {
            std::stringstream ss;
            obj.printAsYAML(ss);
            return ss.str();
          },
          "Dump YAML to string")
      .def(
          "save_to_file",
          [](const yaml& o, const std::string& fileName)
          {
            std::ofstream f(fileName);
            if (!f.is_open())
            {
              throw std::runtime_error("Cannot open file: " + fileName);
            }
            o.printAsYAML(f);
          },
          py::arg("fileName"), "Save YAML to a file")

      // --- Type queries ---
      .def("isNullNode", &yaml::isNullNode, "True if this node is null/empty")
      .def("isScalar", &yaml::isScalar, "True if this node holds a scalar value")
      .def("isMap", &yaml::isMap, "True if this node is a map (dict-like)")
      .def("isSequence", &yaml::isSequence, "True if this node is a sequence (list-like)")

      // --- Basic API ---
      .def("has", &yaml::has, "Check if a key exists in a map node")
      .def("clear", &yaml::clear, "Clear all contents")
      .def("size", &yaml::size, "Number of elements")
      .def("empty", &yaml::empty, "True if node is empty")

      // --- Typed getters (with defaults) ---
      .def(
          "get_str",
          [](const yaml& y, const std::string& key, const std::string& def)
          { return y.getOrDefault<std::string>(key, def); },
          py::arg("key"), py::arg("default") = std::string{},
          "Get string value for key, with optional default")
      .def(
          "get_float",
          [](const yaml& y, const std::string& key, double def)
          { return y.getOrDefault<double>(key, def); },
          py::arg("key"), py::arg("default") = 0.0,
          "Get float value for key, with optional default")
      .def(
          "get_int",
          [](const yaml& y, const std::string& key, int def)
          { return y.getOrDefault<int>(key, def); },
          py::arg("key"), py::arg("default") = 0, "Get int value for key, with optional default")
      .def(
          "get_bool",
          [](const yaml& y, const std::string& key, bool def)
          { return y.getOrDefault<bool>(key, def); },
          py::arg("key"), py::arg("default") = false,
          "Get bool value for key, with optional default")

      // --- Scalar value extractors ---
      .def(
          "as_str", [](const yaml& y) { return y.as<std::string>(); },
          "Extract scalar value as string")
      .def(
          "as_float", [](const yaml& y) { return y.as<double>(); }, "Extract scalar value as float")
      .def(
          "as_int", [](const yaml& y) { return y.as<int>(); }, "Extract scalar value as int")
      .def(
          "as_bool", [](const yaml& y) { return y.as<bool>(); }, "Extract scalar value as bool")

      // --- Sequence operations ---
      .def(
          "push_back", [](yaml& y, double v) { y.push_back(v); }, py::arg("value"),
          "Append a numeric value to a sequence node")
      .def(
          "push_back_str", [](yaml& y, const std::string& v) { y.push_back(v); }, py::arg("value"),
          "Append a string value to a sequence node")

      // --- Map key enumeration ---
      .def(
          "keys",
          [](const yaml& y) -> std::vector<std::string>
          {
            std::vector<std::string> result;
            for (const auto& kv : y.asMapRange())
            {
              result.push_back(static_cast<std::string>(kv.first.internalAsStr()));
            }
            return result;
          },
          "Return list of map keys")

      // --- Python mapping protocol ---
      // Returns a child YAML node (not a native Python type — call as_str/as_float/etc on it).
      .def(
          "__getitem__",
          [](const yaml& y, const std::string& key) -> yaml
          {
            if (!y.has(key))
            {
              throw py::key_error(key);
            }
            return y[key];
          },
          py::arg("key"), "Access child map node by key (returns YAML; call as_str/as_float etc)")
      .def(
          "__setitem__", [](yaml& y, const std::string& key, const yaml& val) { y[key] = val; },
          py::arg("key"), py::arg("value"))
      .def(
          "__contains__", [](const yaml& y, const std::string& key) { return y.has(key); },
          py::arg("key"), "Support Python 'in' operator")
      .def("__len__", [](const yaml& y) { return y.size(); })
      .def(
          "__iter__",
          [](const yaml& y) -> py::object
          {
            if (!y.isMap())
            {
              throw py::type_error("YAML node is not a map — cannot iterate keys");
            }
            std::vector<std::string> keys;
            for (const auto& kv : y.asMapRange())
            {
              keys.push_back(static_cast<std::string>(kv.first.internalAsStr()));
            }
            return py::iter(py::cast(keys));
          },
          "Iterate over map keys")
      .def(
          "__repr__",
          [](const yaml& y)
          {
            std::stringstream ss;
            y.printAsYAML(ss);
            return ss.str();
          });
}
