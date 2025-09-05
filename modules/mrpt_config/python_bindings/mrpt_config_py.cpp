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
#include <mrpt/config/CConfigFile.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/config/CLoadableOptions.h>
#include <mrpt/config/config_parser.h>

namespace py = pybind11;

PYBIND11_MODULE(_bindings, m)
{
  //
  m.doc() = "Python bindings for mrpt_config";

  // Bind CLoadableOptions
  py::class_<mrpt::config::CLoadableOptions>(m, "CLoadableOptions")
      .def(
          "loadFromConfigFile", &mrpt::config::CLoadableOptions::loadFromConfigFile,
          "Loads options from a configuration file section.")
      .def(
          "loadFromConfigFileName", &mrpt::config::CLoadableOptions::loadFromConfigFileName,
          "Loads options directly from a file name.")
      .def(
          "saveToConfigFile", &mrpt::config::CLoadableOptions::saveToConfigFile,
          "Saves options to a configuration file section.")
      .def(
          "saveToConfigFileName", &mrpt::config::CLoadableOptions::saveToConfigFileName,
          "Saves options directly to a file name.")
      .def(
          "dumpToConsole", &mrpt::config::CLoadableOptions::dumpToConsole,
          "Dumps options to the console.")
      .def(
          "dumpToTextStream", &mrpt::config::CLoadableOptions::dumpToTextStream,
          "Dumps options to a text stream.");

  // Bind CConfigFileBase
  py::class_<mrpt::config::CConfigFileBase>(m, "CConfigFileBase")
      .def(
          "getAllSections", &mrpt::config::CConfigFileBase::sections,
          "Returns a list with all section names.")
      .def(
          "getAllKeys", &mrpt::config::CConfigFileBase::keys,
          "Returns a list with all keys in a section.")
      .def(
          "sectionExists", &mrpt::config::CConfigFileBase::sectionExists,
          "Checks if a section exists.")
      .def(
          "keyExists", &mrpt::config::CConfigFileBase::keyExists,
          "Checks if a key exists in a section.")
      .def(
          "setContentFromYAML", &mrpt::config::CConfigFileBase::setContentFromYAML,
          "Sets content from a YAML block.")
      .def(
          "getContentAsYAML", &mrpt::config::CConfigFileBase::getContentAsYAML,
          "Returns content as a YAML block.")
      .def("clear", &mrpt::config::CConfigFileBase::clear, "Empties the config file.")
      .def(
          "write",
          static_cast<void (mrpt::config::CConfigFileBase::*)(
              const std::string&, const std::string&, double, const int, const int,
              const std::string&)>(&mrpt::config::CConfigFileBase::write),
          py::arg("section"), py::arg("name"), py::arg("value"), py::arg("name_padding_width") = -1,
          py::arg("value_padding_width") = -1, py::arg("comment") = "")
      .def(
          "write",
          static_cast<void (mrpt::config::CConfigFileBase::*)(
              const std::string&, const std::string&, const std::string&, const int, const int,
              const std::string&)>(&mrpt::config::CConfigFileBase::write),
          py::arg("section"), py::arg("name"), py::arg("value"), py::arg("name_padding_width") = -1,
          py::arg("value_padding_width") = -1, py::arg("comment") = "")
      .def(
          "read_double", &mrpt::config::CConfigFileBase::read_double, py::arg("section"),
          py::arg("name"), py::arg("defValue"), py::arg("failIfNotFound") = false,
          "Reads a double value with an optional default value.")
      .def(
          "read_float", &mrpt::config::CConfigFileBase::read_float, py::arg("section"),
          py::arg("name"), py::arg("defValue"), py::arg("failIfNotFound") = false,
          "Reads a float value with an optional default value.")
      .def(
          "read_bool", &mrpt::config::CConfigFileBase::read_bool, py::arg("section"),
          py::arg("name"), py::arg("defValue"), py::arg("failIfNotFound") = false,
          "Reads a bool value with an optional default value.")
      .def(
          "read_int", &mrpt::config::CConfigFileBase::read_int, py::arg("section"), py::arg("name"),
          py::arg("defValue"), py::arg("failIfNotFound") = false,
          "Reads an integer value with an optional default value.")
      .def(
          "read_uint64_t", &mrpt::config::CConfigFileBase::read_uint64_t, py::arg("section"),
          py::arg("name"), py::arg("defValue"), py::arg("failIfNotFound") = false,
          "Reads a 64-bit unsigned integer value with an optional default value.")
      .def(
          "read_string", &mrpt::config::CConfigFileBase::read_string, py::arg("section"),
          py::arg("name"), py::arg("defValue") = "", py::arg("failIfNotFound") = false,
          "Reads a string value with an optional default value.")
      .def(
          "read_string_first_word", &mrpt::config::CConfigFileBase::read_string_first_word,
          py::arg("section"), py::arg("name"), py::arg("defValue") = "",
          py::arg("failIfNotFound") = false,
          "Reads the first word of a string value with an optional default value.");

  // Bind CConfigFile, inheriting from CConfigFileBase
  py::class_<mrpt::config::CConfigFile, mrpt::config::CConfigFileBase>(m, "CConfigFile")
      .def(py::init<const std::string&>(), "Constructor for a file.")
      .def(py::init<>(), "Empty constructor.")
      .def(
          "setFileName", &mrpt::config::CConfigFile::setFileName,
          "Associates the object with a file.")
      .def(
          "writeNow", &mrpt::config::CConfigFile::writeNow,
          "Writes changes to the physical file immediately.")
      .def(
          "discardSavingChanges", &mrpt::config::CConfigFile::discardSavingChanges,
          "Discards saving changes to the physical file.")
      .def(
          "getAssociatedFile", &mrpt::config::CConfigFile::getAssociatedFile,
          "Returns the associated file name.");

  // Bind CConfigFileMemory, inheriting from CConfigFileBase
  py::class_<mrpt::config::CConfigFileMemory, mrpt::config::CConfigFileBase>(m, "CConfigFileMemory")
      .def(py::init<>(), "Empty constructor.")
      .def(py::init<const std::vector<std::string>&>(), "Constructor with a list of strings.")
      .def(py::init<const std::string&>(), "Constructor with a single string.")
      .def(
          "getContent", &mrpt::config::CConfigFileMemory::getContent,
          "Returns the current content.")
      .def(
          "setContent",
          static_cast<void (mrpt::config::CConfigFileMemory::*)(const std::vector<std::string>&)>(
              &mrpt::config::CConfigFileMemory::setContent),
          "Sets content from a list of strings.")
      .def(
          "setContent",
          static_cast<void (mrpt::config::CConfigFileMemory::*)(const std::string&)>(
              &mrpt::config::CConfigFileMemory::setContent),
          "Sets content from a single string.");

  // Bind global functions
  m.def("config_parser", &mrpt::config::config_parser, "Parses a configuration document.");
  m.def(
      "MRPT_SAVE_NAME_PADDING", &mrpt::config::MRPT_SAVE_NAME_PADDING,
      "Default padding for names.");
  m.def(
      "MRPT_SAVE_VALUE_PADDING", &mrpt::config::MRPT_SAVE_VALUE_PADDING,
      "Default padding for values.");
}