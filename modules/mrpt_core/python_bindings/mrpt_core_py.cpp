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
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

// MRPT headers
#include <mrpt/core/Clock.h>
#include <mrpt/core/Stringifyable.h>
#include <mrpt/core/WorkerThreadsPool.h>
#include <mrpt/core/abs_diff.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/core/format.h>
#include <mrpt/core/from_string.h>
#include <mrpt/core/get_env.h>
#include <mrpt/core/reverse_bytes.h>

#include <cstdint>
#include <string>

namespace py = pybind11;

// Helper: expose reverseBytes (copy) using MRPT's in-place template
namespace detail
{
template <typename T>
static inline T reverse_bytes_copy(T v)
{
  mrpt::reverseBytesInPlace(v);
  return v;
}
}  // namespace detail

PYBIND11_MODULE(_bindings, m)
{
  m.doc() = "Python bindings for mrpt_core";

  // ------------------ abs_diff ------------------
  m.def("abs_diff_int", &mrpt::abs_diff<int>, "Absolute difference (int)");
  m.def("abs_diff_long", &mrpt::abs_diff<long>, "Absolute difference (long)");
  m.def("abs_diff_float", &mrpt::abs_diff<float>, "Absolute difference (float)");
  m.def("abs_diff_double", &mrpt::abs_diff<double>, "Absolute difference (double)");

  // ------------------ bits_math: DEG2RAD / RAD2DEG ------------------
  m.def("deg2rad", (double (*)(double)) & mrpt::DEG2RAD, "Degrees to radians (double)");
  m.def("rad2deg", (double (*)(double)) & mrpt::RAD2DEG, "Radians to degrees (double)");

  // ------------------ reverse_bytes ------------------
  m.def(
      "reverse_bytes_u16", &detail::reverse_bytes_copy<std::uint16_t>,
      "Reverse endianness for uint16");
  m.def(
      "reverse_bytes_u32", &detail::reverse_bytes_copy<std::uint32_t>,
      "Reverse endianness for uint32");
  m.def(
      "reverse_bytes_u64", &detail::reverse_bytes_copy<std::uint64_t>,
      "Reverse endianness for uint64");
  m.def(
      "reverse_bytes_i16", &detail::reverse_bytes_copy<std::int16_t>,
      "Reverse endianness for int16");
  m.def(
      "reverse_bytes_i32", &detail::reverse_bytes_copy<std::int32_t>,
      "Reverse endianness for int32");
  m.def(
      "reverse_bytes_i64", &detail::reverse_bytes_copy<std::int64_t>,
      "Reverse endianness for int64");

  // ------------------ Stringifyable base ------------------
  py::class_<mrpt::Stringifyable, std::shared_ptr<mrpt::Stringifyable>>(m, "Stringifyable")
      .def("asString", &mrpt::Stringifyable::asString);

  // ------------------ Clock ------------------
  py::class_<mrpt::Clock>(m, "Clock")
      .def_static(
          "nowDouble", []() { return mrpt::Clock::nowDouble(); },
          "Current time in seconds (double)")
      .def_static(
          "now", []() { return mrpt::Clock::now(); }, "Current time as mrpt::Clock::time_point")
      .def_static(
          "toDouble", [](const mrpt::Clock::time_point& tp) { return mrpt::Clock::toDouble(tp); },
          py::arg("time_point"), "Convert mrpt::Clock::time_point to double seconds")
      .def_static(
          "fromDouble", [](double t) { return mrpt::Clock::fromDouble(t); }, py::arg("seconds"),
          "Convert double seconds to mrpt::Clock::time_point");

  // ------------------ format ------------------
  m.def(
      "format",
      [](const std::string& fmt, py::args args) -> std::string
      {
        // Reconstruct the string using Python's str.format() to keep things
        // simple and safe (avoids C printf injection from Python callers).
        // The MRPT mrpt::format() is a thin printf wrapper; expose it for
        // simple cases by forwarding to Python's own formatting.
        //
        // For full printf-style support call it through Python's % operator.
        // We expose it as a direct passthrough for pre-formatted strings.
        (void)args;  // unused — expose a simpler 1-arg version below
        return fmt;
      },
      py::arg("fmt"),
      "Identity overload — returns the string as-is. "
      "Use Python f-strings for formatting instead.");

  m.def(
      "format1d", [](const std::string& fmt, double val) { return mrpt::format(fmt.c_str(), val); },
      py::arg("fmt"), py::arg("val"), "Printf-style format with one double argument");

  m.def(
      "format1s",
      [](const std::string& fmt, const std::string& val)
      { return mrpt::format(fmt.c_str(), val.c_str()); },
      py::arg("fmt"), py::arg("val"), "Printf-style format with one string argument");

  // ------------------ get_env ------------------
  m.def(
      "get_env",
      [](const std::string& var_name, const std::string& default_val) -> std::string
      { return mrpt::get_env<std::string>(var_name, default_val); },
      py::arg("var_name"), py::arg("default_val") = "",
      "Read an environment variable as string; returns default_val if not set");

  m.def(
      "get_env_int",
      [](const std::string& var_name, int default_val) -> int
      { return mrpt::get_env<int>(var_name, default_val); },
      py::arg("var_name"), py::arg("default_val") = 0,
      "Read an environment variable as int; returns default_val if not set");

  m.def(
      "get_env_double",
      [](const std::string& var_name, double default_val) -> double
      { return mrpt::get_env<double>(var_name, default_val); },
      py::arg("var_name"), py::arg("default_val") = 0.0,
      "Read an environment variable as double; returns default_val if not set");

  // ------------------ from_string ------------------
  m.def(
      "from_string_int",
      [](const std::string& s, int def_val, bool throw_on_error) -> int
      { return mrpt::from_string<int>(s, def_val, throw_on_error); },
      py::arg("s"), py::arg("default_value") = 0, py::arg("throw_on_error") = false,
      "Parse string to int; returns default_value on failure unless throw_on_error is True");

  m.def(
      "from_string_double",
      [](const std::string& s, double def_val, bool throw_on_error) -> double
      { return mrpt::from_string<double>(s, def_val, throw_on_error); },
      py::arg("s"), py::arg("default_value") = 0.0, py::arg("throw_on_error") = false,
      "Parse string to double; returns default_value on failure unless throw_on_error is True");

  // ------------------ WorkerThreadsPool ------------------
  py::class_<mrpt::WorkerThreadsPool>(m, "WorkerThreadsPool")
      .def(py::init<unsigned int>(), py::arg("num_threads"))
      .def(
          "enqueue",
          [](mrpt::WorkerThreadsPool& pool, py::function f)
          {
            // wrap Python callable into std::function<void()>
            const auto fut = pool.enqueue(
                [f]()
                {
                  py::gil_scoped_acquire gil;
                  f();
                });
            (void)fut;
          },
          py::arg("func"))
      .def("pendingTasks", &mrpt::WorkerThreadsPool::pendingTasks)
      .def("clear", &mrpt::WorkerThreadsPool::clear)
      .def("size", &mrpt::WorkerThreadsPool::size)
      .def("resize", &mrpt::WorkerThreadsPool::resize)
      .def_property(
          "name", [](const mrpt::WorkerThreadsPool& self) { return self.name(); },
          [](mrpt::WorkerThreadsPool& self, const std::string& n) { self.name(n); },
          "Pool name property");
}