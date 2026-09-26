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

#include <mrpt/io/CCompressedInputStream.h>
#include <mrpt/io/CCompressedOutputStream.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/io/CFileInputStream.h>
#include <mrpt/io/CFileOutputStream.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/io/CStream.h>
#include <mrpt/io/compression_options.h>
#include <mrpt/io/detect_compression.h>
#include <mrpt/io/open_flags.h>
#include <mrpt/io/vector_loadsave.h>
#include <mrpt/io/zip.h>
#include <mrpt/serialization/CArchive.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace pybind11::literals;

namespace
{
std::vector<uint8_t> bytesToVector(const py::bytes& b)
{
  const auto sv = static_cast<std::string_view>(b);
  return {sv.begin(), sv.end()};
}

py::bytes vectorToBytes(const std::vector<uint8_t>& v)
{
  return {reinterpret_cast<const char*>(v.data()), v.size()};
}
}  // namespace

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

  // -------------------------------------------------------------------------
  // vector_loadsave free functions
  // -------------------------------------------------------------------------
  m.def(
      "loadBinaryFile",
      [](const std::string& fileName) { return mrpt::io::loadBinaryFile(fileName); }, "fileName"_a,
      "Load an entire binary file. Returns None (nullopt) on error, or a bytes-like list of "
      "uint8.");
  m.def(
      "loadTextFile", [](const std::string& fileName) { return mrpt::io::loadTextFile(fileName); },
      "fileName"_a, "Load a text file as a list of string lines. Returns None (nullopt) on error.");
  m.def(
      "file_get_contents", &mrpt::io::file_get_contents, "fileName"_a,
      "Load an entire text file as a single string. Raises on error.");
  m.def(
      "vectorToBinaryFile",
      [](const py::bytes& data, const std::string& fileName)
      { return mrpt::io::vectorToBinaryFile(bytesToVector(data), fileName); },
      "data"_a, "fileName"_a, "Save a bytes object to a binary file. Returns False on error.");

  // -------------------------------------------------------------------------
  // Compression
  // -------------------------------------------------------------------------
  py::enum_<mrpt::io::CompressionType>(m, "CompressionType")
      .value("None_", mrpt::io::CompressionType::None)
      .value("Gzip", mrpt::io::CompressionType::Gzip)
      .value("Zstd", mrpt::io::CompressionType::Zstd);

  py::class_<mrpt::io::CompressionOptions>(m, "CompressionOptions")
      .def(py::init<>())
      .def(py::init<mrpt::io::CompressionType, int>(), "type"_a, "level"_a = 1)
      .def_readwrite("type", &mrpt::io::CompressionOptions::type)
      .def_readwrite("level", &mrpt::io::CompressionOptions::level)
      .def(
          "__repr__",
          [](const mrpt::io::CompressionOptions& o)
          {
            return "CompressionOptions(type=" + std::to_string(static_cast<int>(o.type)) +
                   ", level=" + std::to_string(o.level) + ")";
          });

  m.def(
      "detect_compression", &mrpt::io::detect_compression, "filePath"_a,
      "Detects the compression of a file from its magic bytes (not its extension).");

  m.def(
      "archiveFrom", [](mrpt::io::CStream& s) { return mrpt::serialization::archivePtrFrom(s); },
      "stream"_a, py::keep_alive<0, 1>(),
      "Returns a CArchive to read/write MRPT objects from/to the given stream. The stream is "
      "kept alive while the archive exists.");

  // -------------------------------------------------------------------------
  // CCompressedInputStream: reads plain, gzip or zstd files (auto-detected)
  // -------------------------------------------------------------------------
  py::class_<mrpt::io::CCompressedInputStream, mrpt::io::CStream>(m, "CCompressedInputStream")
      .def(py::init<>())
      .def(py::init<const std::string&>(), "fileName"_a)
      .def(
          "open",
          [](mrpt::io::CCompressedInputStream& s, const std::string& fn) { return s.open(fn); },
          "fileName"_a, "Open a plain, gzip or zstd file for reading. Returns true on success.")
      .def("close", &mrpt::io::CCompressedInputStream::close)
      .def("is_open", &mrpt::io::CCompressedInputStream::is_open)
      .def("fileOpenCorrectly", &mrpt::io::CCompressedInputStream::fileOpenCorrectly)
      .def("checkEOF", &mrpt::io::CCompressedInputStream::checkEOF)
      .def("filePathAtUse", &mrpt::io::CCompressedInputStream::filePathAtUse)
      .def("getCompressionType", &mrpt::io::CCompressedInputStream::getCompressionType)
      .def("getTotalBytesCount", &mrpt::io::CCompressedInputStream::getTotalBytesCount)
      .def("getPosition", &mrpt::io::CCompressedInputStream::getPosition)
      .def("getStreamDescription", &mrpt::io::CCompressedInputStream::getStreamDescription)
      .def(
          "__enter__",
          [](mrpt::io::CCompressedInputStream& s) -> mrpt::io::CCompressedInputStream&
          { return s; })
      .def(
          "__exit__", [](mrpt::io::CCompressedInputStream& s, py::object, py::object, py::object)
          { s.close(); })
      .def(
          "__repr__", [](const mrpt::io::CCompressedInputStream& s)
          { return "CCompressedInputStream(" + s.getStreamDescription() + ")"; });

  // -------------------------------------------------------------------------
  // CCompressedOutputStream: writes plain, gzip or zstd files
  // -------------------------------------------------------------------------
  py::class_<mrpt::io::CCompressedOutputStream, mrpt::io::CStream>(m, "CCompressedOutputStream")
      .def(py::init<>())
      .def(
          py::init<const std::string&, mrpt::io::OpenMode, const mrpt::io::CompressionOptions&>(),
          "fileName"_a, "mode"_a = mrpt::io::OpenMode::TRUNCATE,
          "options"_a = mrpt::io::CompressionOptions())
      .def(
          "open",
          [](mrpt::io::CCompressedOutputStream& s, const std::string& fn,
             const mrpt::io::CompressionOptions& options, mrpt::io::OpenMode mode)
          { return s.open(fn, options, std::nullopt, mode); },
          "fileName"_a, "options"_a = mrpt::io::CompressionOptions(),
          "mode"_a = mrpt::io::OpenMode::TRUNCATE,
          "Open a file for writing (default: zstd). Returns true on success.")
      .def("close", &mrpt::io::CCompressedOutputStream::close)
      .def("is_open", &mrpt::io::CCompressedOutputStream::is_open)
      .def("fileOpenCorrectly", &mrpt::io::CCompressedOutputStream::fileOpenCorrectly)
      .def("filePathAtUse", &mrpt::io::CCompressedOutputStream::filePathAtUse)
      .def("getCompressionType", &mrpt::io::CCompressedOutputStream::getCompressionType)
      .def("getPosition", &mrpt::io::CCompressedOutputStream::getPosition)
      .def("getStreamDescription", &mrpt::io::CCompressedOutputStream::getStreamDescription)
      .def(
          "__enter__",
          [](mrpt::io::CCompressedOutputStream& s) -> mrpt::io::CCompressedOutputStream&
          { return s; })
      .def(
          "__exit__", [](mrpt::io::CCompressedOutputStream& s, py::object, py::object, py::object)
          { s.close(); })
      .def(
          "__repr__", [](const mrpt::io::CCompressedOutputStream& s)
          { return "CCompressedOutputStream(" + s.getStreamDescription() + ")"; });

  auto zip = m.def_submodule("zip", "gzip compression of memory blocks and files");
  zip.def(
      "compress_gz_data_block",
      [](const py::bytes& data, int level)
      {
        std::vector<uint8_t> out;
        if (!mrpt::io::zip::compress_gz_data_block(bytesToVector(data), out, level))
        {
          throw std::runtime_error("compress_gz_data_block() failed");
        }
        return vectorToBytes(out);
      },
      "data"_a, "compress_level"_a = 9,
      "Compress a bytes object into a gzip-format bytes object (level 0-9).");
  zip.def(
      "decompress_gz_data_block",
      [](const py::bytes& data)
      {
        std::vector<uint8_t> out;
        if (!mrpt::io::zip::decompress_gz_data_block(bytesToVector(data), out))
        {
          throw std::runtime_error("decompress_gz_data_block() failed");
        }
        return vectorToBytes(out);
      },
      "data"_a,
      "Decompress a gzip-format bytes object. Data not in gzip format is returned unmodified.");
  zip.def(
      "compress_gz_file",
      [](const std::string& filePath, const py::bytes& data, int level)
      { return mrpt::io::zip::compress_gz_file(filePath, bytesToVector(data), level); },
      "file_path"_a, "data"_a, "compress_level"_a = 9,
      "Write a bytes object into a gzip file. Returns False on error.");
  zip.def(
      "decompress_gz_file",
      [](const std::string& filePath) -> std::optional<py::bytes>
      {
        std::vector<uint8_t> out;
        if (!mrpt::io::zip::decompress_gz_file(filePath, out))
        {
          return std::nullopt;
        }
        return vectorToBytes(out);
      },
      "file_path"_a,
      "Read a gzip file (or a plain file, unmodified) into bytes. Returns None on error.");
}
