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

// ---------- recursive to_dict / to_list helpers ----------
static py::object yaml_to_python(const yaml& y);

static py::object yaml_to_python(const yaml& y)
{
  if (y.isNullNode()) return py::none();

  if (y.isScalar())
  {
    // scalar_t is now variant<monostate,bool,int64_t,uint64_t,double,string,shared_ptr<yaml>>
    if (y.scalarType() == typeid(bool)) return py::bool_(y.as<bool>());
    if (y.scalarType() == typeid(int64_t)) return py::int_(y.as<int64_t>());
    if (y.scalarType() == typeid(uint64_t)) return py::int_(y.as<uint64_t>());
    if (y.scalarType() == typeid(double)) return py::float_(y.as<double>());
    return py::str(y.as<std::string>());
  }

  if (y.isMap())
  {
    py::dict d;
    for (const auto& kv : y.asMap())
    {
      const std::string k = static_cast<std::string>(kv.first.internalAsStr());
      d[py::str(k)] = yaml_to_python(yaml(kv.second));  // deep-copy child
    }
    return d;
  }

  if (y.isSequence())
  {
    py::list lst;
    for (const auto& elem : y.asSequence())
    {
      lst.append(yaml_to_python(yaml(elem)));  // deep-copy child
    }
    return lst;
  }

  return py::none();
}

static yaml yaml_from_python(const py::object& obj)
{
  if (obj.is_none()) return yaml();

  if (py::isinstance<py::bool_>(obj))
  {
    yaml y;
    y = obj.cast<bool>();
    return y;
  }
  if (py::isinstance<py::int_>(obj))
  {
    yaml y;
    y = obj.cast<int64_t>();
    return y;
  }
  if (py::isinstance<py::float_>(obj))
  {
    yaml y;
    y = obj.cast<double>();
    return y;
  }
  if (py::isinstance<py::str>(obj))
  {
    yaml y;
    y = obj.cast<std::string>();
    return y;
  }
  if (py::isinstance<py::dict>(obj))
  {
    yaml y = yaml(yaml::Map());
    const auto d = obj.cast<py::dict>();
    for (const auto& kv : d)
    {
      y[kv.first.cast<std::string>()] = yaml_from_python(kv.second.cast<py::object>());
    }
    return y;
  }
  if (py::isinstance<py::list>(obj) || py::isinstance<py::tuple>(obj))
  {
    yaml y = yaml(yaml::Sequence());
    for (const auto& item : obj)
    {
      y.push_back(yaml_from_python(item.cast<py::object>()));
    }
    return y;
  }
  // fallback: try str
  yaml y;
  y = py::str(obj).cast<std::string>();
  return y;
}

PYBIND11_MODULE(_bindings, m)
{
  m.doc() = "Python bindings for mrpt_containers";

  py::class_<yaml>(
      m, "YAML",
      R"doc(
Powerful YAML/JSON container for nested structured data.

**Thread safety:** Not thread-safe. Concurrent reads are safe when no writer
is active; any concurrent write requires external synchronization (same policy
as std::map / std::vector).
)doc")
      .def(py::init<>())

      // --- Static constructors ---
      .def_static("from_string", &yaml::FromText, "Parse YAML/JSON from string")
      .def_static("from_file", &yaml::FromFile, "Parse YAML/JSON from file")
      .def_static(
          "from_dict", [](const py::dict& d) { return yaml_from_python(d); }, py::arg("d"),
          "Build a YAML map node from a Python dict (recursive)")
      .def_static(
          "from_list", [](const py::list& lst) { return yaml_from_python(lst); }, py::arg("lst"),
          "Build a YAML sequence node from a Python list (recursive)")

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
      .def(
          "to_dict",
          [](const yaml& y) -> py::object
          {
            if (!y.isMap())
              throw std::runtime_error(
                  "to_dict() requires a map node, got: " + y.node().typeName());
            return yaml_to_python(y);
          },
          "Recursively convert a map node to a Python dict")
      .def(
          "to_list",
          [](const yaml& y) -> py::object
          {
            if (!y.isSequence())
              throw std::runtime_error(
                  "to_list() requires a sequence node, got: " + y.node().typeName());
            return yaml_to_python(y);
          },
          "Recursively convert a sequence node to a Python list")

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
          "as_int", [](const yaml& y) { return y.as<int64_t>(); }, "Extract scalar value as int")
      .def(
          "as_bool", [](const yaml& y) { return y.as<bool>(); }, "Extract scalar value as bool")

      // --- Sequence operations ---
      .def(
          "push_back", [](yaml& y, double v) { y.push_back(v); }, py::arg("value"),
          "Append a numeric value to a sequence node")
      .def(
          "push_back_str", [](yaml& y, const std::string& v) { y.push_back(v); }, py::arg("value"),
          "Append a string value to a sequence node")
      .def(
          "append", [](yaml& y, const py::object& obj) { y.push_back(yaml_from_python(obj)); },
          py::arg("value"), "Append any Python value (str/int/float/bool/dict/list) to a sequence")

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
      .def(
          "values",
          [](const yaml& y) -> py::list
          {
            py::list result;
            for (const auto& kv : y.asMap())
            {
              result.append(yaml(kv.second));  // deep-copy each value node
            }
            return result;
          },
          "Return list of map values as YAML nodes")
      .def(
          "items",
          [](const yaml& y) -> py::list
          {
            py::list result;
            for (const auto& kv : y.asMap())
            {
              const std::string k = static_cast<std::string>(kv.first.internalAsStr());
              result.append(py::make_tuple(k, yaml(kv.second)));  // deep-copy
            }
            return result;
          },
          "Return list of (key, YAML) pairs")

      // --- Python mapping protocol ---
      .def(
          "__getitem__",
          [](const yaml& y, const py::object& key_obj) -> yaml
          {
            if (py::isinstance<py::str>(key_obj))
            {
              const auto key = key_obj.cast<std::string>();
              if (!y.has(key)) throw py::key_error(key);
              return y[key];
            }
            if (py::isinstance<py::int_>(key_obj))
            {
              const int idx = key_obj.cast<int>();
              return y[idx];
            }
            throw py::type_error("YAML key must be str or int");
          },
          py::arg("key"), "Access child node by string key (map) or int index (sequence)")
      .def(
          "__setitem__",
          [](yaml& y, const py::object& key_obj, const py::object& val_obj)
          {
            if (py::isinstance<py::str>(key_obj))
            {
              const auto key = key_obj.cast<std::string>();
              y[key] = yaml_from_python(val_obj);
              return;
            }
            if (py::isinstance<py::int_>(key_obj))
            {
              const int idx = key_obj.cast<int>();
              y[idx] = yaml_from_python(val_obj);
              return;
            }
            throw py::type_error("YAML key must be str or int");
          },
          py::arg("key"), py::arg("value"))
      .def(
          "__delitem__",
          [](yaml& y, const py::object& key_obj)
          {
            if (py::isinstance<py::str>(key_obj))
            {
              const auto key = key_obj.cast<std::string>();
              if (y.erase(key) == 0) throw py::key_error(key);
              return;
            }
            if (py::isinstance<py::int_>(key_obj))
            {
              const int idx = key_obj.cast<int>();
              if (!y.erase(idx)) throw py::index_error(std::to_string(idx));
              return;
            }
            throw py::type_error("YAML key must be str or int");
          },
          py::arg("key"), "Delete a map entry by key or sequence element by index")
      .def(
          "__contains__", [](const yaml& y, const std::string& key) { return y.has(key); },
          py::arg("key"), "Support Python 'in' operator for map keys")
      .def("__len__", [](const yaml& y) { return y.size(); })
      .def(
          "__iter__",
          [](const yaml& y) -> py::object
          {
            if (y.isMap())
            {
              std::vector<std::string> keys;
              for (const auto& kv : y.asMapRange())
              {
                keys.push_back(static_cast<std::string>(kv.first.internalAsStr()));
              }
              return py::iter(py::cast(keys));
            }
            if (y.isSequence())
            {
              py::list lst;
              for (const auto& elem : y.asSequence())
              {
                lst.append(yaml(elem));  // deep-copy
              }
              return py::iter(lst);
            }
            throw py::type_error("YAML node is not iterable (not a map or sequence)");
          },
          "Iterate over map keys (for maps) or child nodes (for sequences)")
      .def(
          "__repr__",
          [](const yaml& y)
          {
            std::stringstream ss;
            mrpt::containers::YamlEmitOptions eo;
            eo.emitHeader = false;
            eo.endWithNewLine = false;
            y.printAsYAML(ss, eo);
            return "YAML(" + ss.str() + ")";
          })
      .def(
          "__str__",
          [](const yaml& y)
          {
            std::stringstream ss;
            y.printAsYAML(ss);
            return ss.str();
          });
}
