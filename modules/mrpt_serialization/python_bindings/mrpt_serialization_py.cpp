/* -------------------------------------------------------------------------
 * Mobile Robot Programming Toolkit (MRPT)
 * https://github.com/MRPT/mrpt/
 * ------------------------------------------------------------------------- */

#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/serialization/archiveFrom_std_vector.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_MODULE(_bindings, m)
{
  m.doc() = "Python bindings for mrpt_serialization";

  // End of stream while reading objects maps to Python's EOFError, so
  // reading loops can stop with `except EOFError`.
  py::register_exception<mrpt::serialization::CExceptionEOF>(m, "CExceptionEOF", PyExc_EOFError);

  // 1. CSerializable
  // Note: py::base<mrpt::rtti::CObject>() tells pybind11 about the
  // inheritance even if CObject is defined in a different module.
  py::class_<
      mrpt::serialization::CSerializable, std::shared_ptr<mrpt::serialization::CSerializable>>(
      m, "CSerializable")
      .def(
          "GetRuntimeClass",
          [](const mrpt::serialization::CSerializable& self) { return self.GetRuntimeClass(); },
          py::return_value_policy::reference);

  // 2. CArchive
  py::class_<mrpt::serialization::CArchive, std::shared_ptr<mrpt::serialization::CArchive>>(
      m, "CArchive")
      // High-level Object I/O
      .def(
          "ReadObject",
          [](mrpt::serialization::CArchive& self)
          { return self.ReadObject<mrpt::serialization::CSerializable>(); },
          "Reads an MRPT object from the stream. Raises EOFError at the end of the stream.")
      .def(
          "ReadObject",
          [](mrpt::serialization::CArchive& self, mrpt::serialization::CSerializable& obj)
          { self.ReadObject(&obj); },
          "obj"_a,
          "Reads an MRPT object from the stream into an existing object, which must be of the "
          "same class.")
      .def(
          "__repr__", [](const mrpt::serialization::CArchive& self)
          { return "CArchive('" + self.getArchiveDescription() + "')"; })

      .def(
          "WriteObject",
          [](mrpt::serialization::CArchive& self,
             const mrpt::serialization::CSerializable::Ptr& obj) { self.WriteObject(obj.get()); },
          "Writes an MRPT object to the stream.")

      // Data Type I/O (using lambdas to resolve templates/protected access)
      .def("ReadDouble", &mrpt::serialization::CArchive::ReadPOD<double>)
      .def("ReadInt", &mrpt::serialization::CArchive::ReadPOD<int32_t>);

  // 3. Global Utility Functions
  // This allows Python users to easily serialize/deserialize to 'bytes'
  m.def(
      "objectToBytes",
      [](const mrpt::serialization::CSerializable::Ptr& obj)
      {
        std::vector<uint8_t> buf;
        mrpt::serialization::ObjectToOctetVector(obj.get(), buf);
        return py::bytes(reinterpret_cast<const char*>(buf.data()), buf.size());
      },
      "Converts an MRPT object to a Python bytes object.");

  m.def(
      "bytesToObject",
      [](const py::bytes& b)
      {
        const auto sv = static_cast<std::string_view>(b);
        std::vector<uint8_t> buf(sv.begin(), sv.end());
        mrpt::serialization::CSerializable::Ptr obj;
        mrpt::serialization::OctetVectorToObject(buf, obj);
        return obj;
      },
      "Converts a Python bytes object back into an MRPT object.");
}