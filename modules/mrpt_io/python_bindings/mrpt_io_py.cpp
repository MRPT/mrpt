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

#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/io/CFileInputStream.h>
#include <mrpt/io/CFileOutputStream.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/io/CStream.h>
#include <mrpt/io/open_flags.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_MODULE(_bindings, m)
{
  m.doc() = "Python bindings for mrpt::io — file and memory streams";

  // -------------------------------------------------------------------------
  // OpenMode enum
  // -------------------------------------------------------------------------
  py::enum_<mrpt::io::OpenMode>(m, "OpenMode")
      .value("TRUNCATE", mrpt::io::OpenMode::TRUNCATE)
      .value("APPEND", mrpt::io::OpenMode::APPEND)
      .export_values();

  // -------------------------------------------------------------------------
  // CStream::TSeekOrigin enum
  // -------------------------------------------------------------------------
  py::enum_<mrpt::io::CStream::TSeekOrigin>(m, "SeekOrigin")
      .value("sFromBeginning", mrpt::io::CStream::sFromBeginning)
      .value("sFromCurrent", mrpt::io::CStream::sFromCurrent)
      .value("sFromEnd", mrpt::io::CStream::sFromEnd)
      .export_values();

  // -------------------------------------------------------------------------
  // CStream — abstract base for all stream types
  // -------------------------------------------------------------------------
  py::class_<mrpt::io::CStream>(m, "CStream")
      .def(
          "Seek",
          [](mrpt::io::CStream& s, int64_t offset, mrpt::io::CStream::TSeekOrigin origin)
          { return s.Seek(offset, origin); },
          "offset"_a, "origin"_a = mrpt::io::CStream::sFromBeginning)
      .def("getTotalBytesCount", &mrpt::io::CStream::getTotalBytesCount)
      .def("getPosition", &mrpt::io::CStream::getPosition)
      .def("getStreamDescription", &mrpt::io::CStream::getStreamDescription)
      .def("getline", &mrpt::io::CStream::getline)
      // Read bytes into a Python bytes object
      .def(
          "read",
          [](mrpt::io::CStream& s, size_t count)
          {
            std::vector<uint8_t> buf(count);
            const size_t n = s.Read(buf.data(), count);
            buf.resize(n);
            return py::bytes(reinterpret_cast<const char*>(buf.data()), n);
          },
          "count"_a, "Read up to `count` bytes from the stream, returns bytes object")
      // Write bytes from a Python bytes/bytearray object
      .def(
          "write",
          [](mrpt::io::CStream& s, const py::bytes& data)
          {
            std::string_view sv = data;
            return s.Write(sv.data(), sv.size());
          },
          "data"_a, "Write bytes to the stream, returns number of bytes written");

  // -------------------------------------------------------------------------
  // CFileInputStream — read-only binary file stream
  // -------------------------------------------------------------------------
  py::class_<mrpt::io::CFileInputStream, mrpt::io::CStream>(m, "CFileInputStream")
      .def(py::init<>())
      .def(py::init<const std::string&>(), "fileName"_a)
      .def(
          "open", &mrpt::io::CFileInputStream::open, "fileName"_a,
          "Open a file for reading. Returns true on success.")
      .def("close", &mrpt::io::CFileInputStream::close)
      .def("is_open", &mrpt::io::CFileInputStream::is_open)
      .def("fileOpenCorrectly", &mrpt::io::CFileInputStream::fileOpenCorrectly)
      .def("checkEOF", &mrpt::io::CFileInputStream::checkEOF)
      .def("clearError", &mrpt::io::CFileInputStream::clearError)
      .def("readLine", &mrpt::io::CFileInputStream::readLine)
      .def("getTotalBytesCount", &mrpt::io::CFileInputStream::getTotalBytesCount)
      .def("getPosition", &mrpt::io::CFileInputStream::getPosition)
      .def("getStreamDescription", &mrpt::io::CFileInputStream::getStreamDescription)
      // Context manager support
      .def(
          "__enter__",
          [](mrpt::io::CFileInputStream& s) -> mrpt::io::CFileInputStream& { return s; })
      .def(
          "__exit__",
          [](mrpt::io::CFileInputStream& s, py::object, py::object, py::object) { s.close(); })
      .def(
          "__repr__", [](const mrpt::io::CFileInputStream& s)
          { return "CFileInputStream(" + s.getStreamDescription() + ")"; });

  // -------------------------------------------------------------------------
  // CFileOutputStream — write-only binary file stream
  // -------------------------------------------------------------------------
  py::class_<mrpt::io::CFileOutputStream, mrpt::io::CStream>(m, "CFileOutputStream")
      .def(py::init<>())
      .def(
          py::init<const std::string&, mrpt::io::OpenMode>(), "fileName"_a,
          "mode"_a = mrpt::io::OpenMode::TRUNCATE)
      .def(
          "open",
          [](mrpt::io::CFileOutputStream& s, const std::string& fn, mrpt::io::OpenMode mode)
          { return s.open(fn, mode); },
          "fileName"_a, "mode"_a = mrpt::io::OpenMode::TRUNCATE,
          "Open a file for writing. Returns true on success.")
      .def("close", &mrpt::io::CFileOutputStream::close)
      .def("is_open", &mrpt::io::CFileOutputStream::is_open)
      .def("fileOpenCorrectly", &mrpt::io::CFileOutputStream::fileOpenCorrectly)
      .def("getTotalBytesCount", &mrpt::io::CFileOutputStream::getTotalBytesCount)
      .def("getPosition", &mrpt::io::CFileOutputStream::getPosition)
      .def("getStreamDescription", &mrpt::io::CFileOutputStream::getStreamDescription)
      // Context manager support
      .def(
          "__enter__",
          [](mrpt::io::CFileOutputStream& s) -> mrpt::io::CFileOutputStream& { return s; })
      .def(
          "__exit__",
          [](mrpt::io::CFileOutputStream& s, py::object, py::object, py::object) { s.close(); })
      .def(
          "__repr__", [](const mrpt::io::CFileOutputStream& s)
          { return "CFileOutputStream(" + s.getStreamDescription() + ")"; });

  // -------------------------------------------------------------------------
  // CFileGZInputStream — transparent gz-compressed input stream
  // -------------------------------------------------------------------------
  py::class_<mrpt::io::CFileGZInputStream, mrpt::io::CStream>(m, "CFileGZInputStream")
      .def(py::init<>())
      .def(py::init<const std::string&>(), "fileName"_a)
      .def(
          "open", [](mrpt::io::CFileGZInputStream& s, const std::string& fn) { return s.open(fn); },
          "fileName"_a, "Open a .gz file for reading. Returns true on success.")
      .def("close", &mrpt::io::CFileGZInputStream::close)
      .def("is_open", &mrpt::io::CFileGZInputStream::is_open)
      .def("fileOpenCorrectly", &mrpt::io::CFileGZInputStream::fileOpenCorrectly)
      .def("checkEOF", &mrpt::io::CFileGZInputStream::checkEOF)
      .def("filePathAtUse", &mrpt::io::CFileGZInputStream::filePathAtUse)
      .def("getTotalBytesCount", &mrpt::io::CFileGZInputStream::getTotalBytesCount)
      .def("getPosition", &mrpt::io::CFileGZInputStream::getPosition)
      .def("getStreamDescription", &mrpt::io::CFileGZInputStream::getStreamDescription)
      // Context manager support
      .def(
          "__enter__",
          [](mrpt::io::CFileGZInputStream& s) -> mrpt::io::CFileGZInputStream& { return s; })
      .def(
          "__exit__",
          [](mrpt::io::CFileGZInputStream& s, py::object, py::object, py::object) { s.close(); })
      .def(
          "__repr__", [](const mrpt::io::CFileGZInputStream& s)
          { return "CFileGZInputStream(" + s.getStreamDescription() + ")"; });

  // -------------------------------------------------------------------------
  // CFileGZOutputStream — transparent gz-compressed output stream
  // -------------------------------------------------------------------------
  py::class_<mrpt::io::CFileGZOutputStream, mrpt::io::CStream>(m, "CFileGZOutputStream")
      .def(py::init<>())
      .def(
          py::init<const std::string&, mrpt::io::OpenMode, int>(), "fileName"_a,
          "mode"_a = mrpt::io::OpenMode::TRUNCATE, "compressionLevel"_a = 1)
      .def(
          "open",
          [](mrpt::io::CFileGZOutputStream& s, const std::string& fn, int level,
             mrpt::io::OpenMode mode) { return s.open(fn, level, std::nullopt, mode); },
          "fileName"_a, "compress_level"_a = 1, "mode"_a = mrpt::io::OpenMode::TRUNCATE,
          "Open a .gz file for writing. Returns true on success.")
      .def("close", &mrpt::io::CFileGZOutputStream::close)
      .def("is_open", &mrpt::io::CFileGZOutputStream::is_open)
      .def("fileOpenCorrectly", &mrpt::io::CFileGZOutputStream::fileOpenCorrectly)
      .def("filePathAtUse", &mrpt::io::CFileGZOutputStream::filePathAtUse)
      .def("getPosition", &mrpt::io::CFileGZOutputStream::getPosition)
      .def("getStreamDescription", &mrpt::io::CFileGZOutputStream::getStreamDescription)
      // Context manager support
      .def(
          "__enter__",
          [](mrpt::io::CFileGZOutputStream& s) -> mrpt::io::CFileGZOutputStream& { return s; })
      .def(
          "__exit__",
          [](mrpt::io::CFileGZOutputStream& s, py::object, py::object, py::object) { s.close(); })
      .def(
          "__repr__", [](const mrpt::io::CFileGZOutputStream& s)
          { return "CFileGZOutputStream(" + s.getStreamDescription() + ")"; });

  // -------------------------------------------------------------------------
  // CMemoryStream — in-memory stream buffer
  // -------------------------------------------------------------------------
  py::class_<mrpt::io::CMemoryStream, mrpt::io::CStream>(m, "CMemoryStream")
      .def(py::init<>())
      .def("clear", &mrpt::io::CMemoryStream::clear)
      .def("getTotalBytesCount", &mrpt::io::CMemoryStream::getTotalBytesCount)
      .def("getPosition", &mrpt::io::CMemoryStream::getPosition)
      .def(
          "Seek",
          [](mrpt::io::CMemoryStream& s, int64_t offset, mrpt::io::CStream::TSeekOrigin origin)
          { return s.Seek(offset, origin); },
          "offset"_a, "origin"_a = mrpt::io::CStream::sFromBeginning)
      .def("saveBufferToFile", &mrpt::io::CMemoryStream::saveBufferToFile)
      .def("loadBufferFromFile", &mrpt::io::CMemoryStream::loadBufferFromFile)
      // Expose as bytes
      .def(
          "getContents",
          [](const mrpt::io::CMemoryStream& s)
          {
            const size_t n = s.getTotalBytesCount();
            return py::bytes(reinterpret_cast<const char*>(s.getRawBufferData()), n);
          },
          "Return a copy of the stream buffer as a Python bytes object")
      .def(
          "__repr__", [](const mrpt::io::CMemoryStream& s)
          { return "CMemoryStream(" + std::to_string(s.getTotalBytesCount()) + " bytes)"; });
}
