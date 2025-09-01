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
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

// MRPT headers
#include <mrpt/system/CTicTac.h>
#include <mrpt/system/CTimeLogger.h>
#include <mrpt/system/crc.h>
#include <mrpt/system/string_utils.h>

namespace py = pybind11;

PYBIND11_MODULE(_bindings, m)
{
  m.doc() = "Python bindings for mrpt_system";

  // crc -------------------------------------------
  // CRC16 overloads
  m.def(
      "compute_CRC16",
      static_cast<uint16_t (*)(const std::vector<uint8_t>&, const uint16_t)>(
          &mrpt::system::compute_CRC16),
      py::arg("data"), py::arg("gen_pol") = 0x8005,
      "Compute CRC16 checksum from a vector of bytes");

  m.def(
      "compute_CRC16",
      [](py::bytes data, uint16_t gen_pol)
      {
        std::string buf = data;  // convert Python bytes to std::string
        return mrpt::system::compute_CRC16(
            reinterpret_cast<const uint8_t*>(buf.data()), buf.size(), gen_pol);
      },
      py::arg("data"), py::arg("gen_pol") = 0x8005, "Compute CRC16 checksum from raw bytes");

  // CRC32 overloads
  m.def(
      "compute_CRC32",
      static_cast<uint32_t (*)(const std::vector<uint8_t>&, const uint32_t)>(
          &mrpt::system::compute_CRC32),
      py::arg("data"), py::arg("gen_pol") = 0xEDB88320L,
      "Compute CRC32 checksum from a vector of bytes");

  m.def(
      "compute_CRC32",
      [](py::bytes data, uint32_t gen_pol)
      {
        std::string buf = data;
        return mrpt::system::compute_CRC32(
            reinterpret_cast<const uint8_t*>(buf.data()), buf.size(), gen_pol);
      },
      py::arg("data"), py::arg("gen_pol") = 0xEDB88320L, "Compute CRC32 checksum from raw bytes");

  // CTicTac -------------------------------------------
  py::class_<mrpt::system::CTicTac>(m, "CTicTac")
      .def(py::init<>(), "Create a new stopwatch and start it automatically")
      .def("Tic", &mrpt::system::CTicTac::Tic, "Start or restart the stopwatch")
      .def(
          "Tac", &mrpt::system::CTicTac::Tac,
          "Stop the stopwatch and return the elapsed time in seconds");

  // string_utils ----------------------------------------

  m.def(
      "encodeBase64",
      [](py::bytes input)
      {
        std::string buf = input;
        std::vector<uint8_t> data(buf.begin(), buf.end());
        std::string out;
        mrpt::system::encodeBase64(data, out);
        return out;  // return as Python str
      },
      py::arg("input"), "Encode a bytes-like object into a Base64 string");

  m.def(
      "decodeBase64",
      [](const std::string& b64str)
      {
        std::vector<uint8_t> out;
        if (!mrpt::system::decodeBase64(b64str, out))
        {
          throw std::runtime_error("Invalid base64 string");
        }
        return py::bytes(reinterpret_cast<const char*>(out.data()), out.size());
      },
      py::arg("input"), "Decode a Base64 string into raw bytes");

  m.def(
      "unitsFormat", &mrpt::system::unitsFormat, py::arg("val"), py::arg("nDecimalDigits") = 2,
      py::arg("middle_space") = true,
      "Format a value with SI metric unit prefixes (e.g., 1e3 -> '1.00 K')");

  // --- CTimeLogger ---
  py::class_<mrpt::system::CTimeLogger> logger(m, "CTimeLogger");

  logger.def(
      py::init<bool, const std::string&, bool>(), py::arg("enabled") = true, py::arg("name") = "",
      py::arg("keep_whole_history") = false, "Construct a CTimeLogger");

  logger.def(
      "enter", &mrpt::system::CTimeLogger::enter, py::arg("section_name"),
      "Start a named section (time measurement)");

  logger.def(
      "leave", &mrpt::system::CTimeLogger::leave, py::arg("section_name"),
      "End a named section and return elapsed time in seconds");

  logger.def(
      "getMeanTime", &mrpt::system::CTimeLogger::getMeanTime, py::arg("section_name"),
      "Return mean execution time of a section");

  logger.def(
      "getLastTime", &mrpt::system::CTimeLogger::getLastTime, py::arg("section_name"),
      "Return last execution time of a section");

  logger.def("enable", &mrpt::system::CTimeLogger::enable, py::arg("enabled") = true);
  logger.def("disable", &mrpt::system::CTimeLogger::disable);
  logger.def("isEnabled", &mrpt::system::CTimeLogger::isEnabled);
  logger.def(
      "enableKeepWholeHistory", &mrpt::system::CTimeLogger::enableKeepWholeHistory,
      py::arg("enable") = true);
  logger.def("isEnabledKeepWholeHistory", &mrpt::system::CTimeLogger::isEnabledKeepWholeHistory);

  logger.def(
      "getStatsAsText", &mrpt::system::CTimeLogger::getStatsAsText, py::arg("column_width") = 80);
  logger.def(
      "dumpAllStats", &mrpt::system::CTimeLogger::dumpAllStats, py::arg("column_width") = 80);
  logger.def("saveToCSVFile", &mrpt::system::CTimeLogger::saveToCSVFile, py::arg("csv_file"));
  logger.def("saveToMFile", &mrpt::system::CTimeLogger::saveToMFile, py::arg("m_file"));
  logger.def("clear", &mrpt::system::CTimeLogger::clear, py::arg("deep_clear") = false);
  logger.def("setName", &mrpt::system::CTimeLogger::setName, py::arg("name"));
  logger.def("getName", &mrpt::system::CTimeLogger::getName, py::return_value_policy::reference);

  // --- CTimeLoggerEntry ---
  py::class_<mrpt::system::CTimeLoggerEntry>(m, "CTimeLoggerEntry")
      .def(
          py::init<mrpt::system::CTimeLogger&, const std::string&>(), py::arg("logger"),
          py::arg("section_name"), "Scoped time logging entry")
      .def("stop", &mrpt::system::CTimeLoggerEntry::stop);

  // --- CTimeLoggerSaveAtDtor ---
  py::class_<mrpt::system::CTimeLoggerSaveAtDtor>(m, "CTimeLoggerSaveAtDtor")
      .def(py::init<mrpt::system::CTimeLogger&>(), py::arg("logger"));

  // --- Global profiler functions ---
  m.def("global_profiler_enter", &mrpt::system::global_profiler_enter, py::arg("func_name"));
  m.def("global_profiler_leave", &mrpt::system::global_profiler_leave, py::arg("func_name"));
  m.def(
      "global_profiler_getref", &mrpt::system::global_profiler_getref,
      py::return_value_policy::reference);
}
