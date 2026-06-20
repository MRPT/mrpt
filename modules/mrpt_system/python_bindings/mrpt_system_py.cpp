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
#include <pybind11/chrono.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

// MRPT headers
#include <mrpt/system/CTicTac.h>
#include <mrpt/system/CTimeLogger.h>
#include <mrpt/system/crc.h>
#include <mrpt/system/datetime.h>
#include <mrpt/system/filesystem.h>
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

  // --- filesystem ---
  m.def(
      "fileExists", &mrpt::system::fileExists, py::arg("fileName"),
      "Returns true if the file exists on disk");
  m.def(
      "directoryExists", &mrpt::system::directoryExists, py::arg("dirName"),
      "Returns true if the directory exists");
  m.def(
      "getTempFileName", &mrpt::system::getTempFileName,
      "Returns the name of a proposed temporary file");
  m.def("getcwd", &mrpt::system::getcwd, "Returns the current working directory");
  m.def(
      "createDirectory", &mrpt::system::createDirectory, py::arg("dirName"),
      "Creates a directory. Returns true on success");
  m.def(
      "deleteFile", &mrpt::system::deleteFile, py::arg("fileName"),
      "Deletes a file. Returns true on success");
  m.def(
      "renameFile",
      [](const std::string& oldName, const std::string& newName) -> bool
      { return mrpt::system::renameFile(oldName, newName); },
      py::arg("oldFileName"), py::arg("newFileName"), "Renames a file. Returns true on success");
  m.def(
      "extractFileName", &mrpt::system::extractFileName, py::arg("filePath"),
      "Extracts just the filename from a full path (no extension, no directory)");
  m.def(
      "extractFileExtension", &mrpt::system::extractFileExtension, py::arg("filePath"),
      py::arg("ignore_gz") = false, "Extracts the file extension (without the dot)");
  m.def(
      "extractFileDirectory", &mrpt::system::extractFileDirectory, py::arg("filePath"),
      "Extracts the directory part of a full file path");
  m.def(
      "fileNameChangeExtension", &mrpt::system::fileNameChangeExtension, py::arg("filename"),
      py::arg("newExtension"), "Returns the filename with the extension changed");
  m.def(
      "fileNameStripInvalidChars", &mrpt::system::fileNameStripInvalidChars, py::arg("filename"),
      py::arg("replace_with") = '_',
      "Replaces characters not valid for a filename with a replacement char");
  m.def(
      "getFileSize", &mrpt::system::getFileSize, py::arg("fileName"),
      "Returns the size of a file in bytes");
  m.def(
      "toAbsolutePath", &mrpt::system::toAbsolutePath, py::arg("path"),
      py::arg("resolveToCanonical") = false,
      "Converts a relative path to absolute, optionally resolving symlinks");
  m.def(
      "pathJoin", &mrpt::system::pathJoin, py::arg("parts"),
      "Joins path components, mirroring Python's os.path.join semantics");
  m.def(
      "filePathSeparatorsToNative", &mrpt::system::filePathSeparatorsToNative, py::arg("filePath"),
      "Converts path separators to the native format");

  // --- datetime ---
  py::class_<mrpt::system::TTimeParts>(
      m, "TTimeParts", "Broken-down date/time representation (UTC or local)")
      .def(py::init<>())
      .def_readwrite("year", &mrpt::system::TTimeParts::year)
      .def_readwrite("month", &mrpt::system::TTimeParts::month)
      .def_readwrite("day", &mrpt::system::TTimeParts::day)
      .def_readwrite("hour", &mrpt::system::TTimeParts::hour)
      .def_readwrite("minute", &mrpt::system::TTimeParts::minute)
      .def_readwrite("second", &mrpt::system::TTimeParts::second)
      .def_readwrite("day_of_week", &mrpt::system::TTimeParts::day_of_week);

  m.def(
      "buildTimestampFromParts", &mrpt::system::buildTimestampFromParts, py::arg("parts"),
      "Build a TTimeStamp (UTC) from a TTimeParts struct");
  m.def(
      "buildTimestampFromPartsLocalTime", &mrpt::system::buildTimestampFromPartsLocalTime,
      py::arg("parts"), "Build a TTimeStamp (local time) from a TTimeParts struct");

  m.def(
      "timestampToParts",
      [](mrpt::system::TTimeStamp t, bool localTime)
      {
        mrpt::system::TTimeParts p;
        mrpt::system::timestampToParts(t, p, localTime);
        return p;
      },
      py::arg("t"), py::arg("localTime") = false,
      "Decomposes a TTimeStamp into a TTimeParts struct (UTC by default)");

  m.def(
      "timeDifference",
      [](mrpt::system::TTimeStamp t_first, mrpt::system::TTimeStamp t_later)
      { return mrpt::system::timeDifference(t_first, t_later); },
      py::arg("t_first"), py::arg("t_later"),
      "Returns the difference in seconds (t_later - t_first)");

  m.def(
      "timestampAdd",
      [](mrpt::system::TTimeStamp t, double seconds)
      { return mrpt::system::timestampAdd(t, seconds); },
      py::arg("t"), py::arg("num_seconds"), "Adds a number of seconds to a timestamp");

  m.def(
      "dateTimeToString", &mrpt::system::dateTimeToString, py::arg("t"),
      "Converts a timestamp to a human-readable UTC date-time string");
  m.def(
      "dateTimeLocalToString", &mrpt::system::dateTimeLocalToString, py::arg("t"),
      "Converts a timestamp to a human-readable local date-time string");
  m.def(
      "dateToString", &mrpt::system::dateToString, py::arg("t"),
      "Converts a timestamp to a date-only string (UTC)");
  m.def(
      "timeToString", &mrpt::system::timeToString, py::arg("t"),
      "Converts a timestamp to a time-only string (UTC)");
  m.def(
      "timeLocalToString", &mrpt::system::timeLocalToString, py::arg("t"),
      py::arg("secondFractionDigits") = 6, "Converts a timestamp to a local time-only string");
  m.def(
      "formatTimeInterval", &mrpt::system::formatTimeInterval, py::arg("timeSeconds"),
      "Formats a time interval (seconds) as a human-readable string (e.g. '1h 23m 45s')");
  m.def(
      "intervalFormat", &mrpt::system::intervalFormat, py::arg("seconds"),
      "Format a time interval in seconds to a string");
}
