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

PYBIND11_MODULE(_bindings, m)
{
  m.doc() = "Python bindings for mrpt_serialization";

  // 1. CSerializable
  // Note: py::base<mrpt::rtti::CObject>() tells pybind11 about the
  // inheritance even if CObject is defined in a different module.
  py::class_<
      mrpt::serialization::CSerializable, std::shared_ptr<mrpt::serialization::CSerializable>>(
      m, "CSerializable")
      .def(
          "GetRuntimeClass",
          [](const mrpt::serialization::CSerializable& self) { return self.GetRuntimeClass(); });

  // 2. CArchive
  py::class_<mrpt::serialization::CArchive, std::shared_ptr<mrpt::serialization::CArchive>>(
      m, "CArchive")
      // High-level Object I/O
      .def(
          "ReadObject",
          [](mrpt::serialization::CArchive& self)
          { return self.ReadObject<mrpt::serialization::CSerializable>(); },
          "Reads an MRPT object from the stream.")

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
      [](const mrpt::serialization::CSerializable& obj)
      {
        std::vector<uint8_t> buf;
        mrpt::serialization::ObjectToOctetVector(&obj, buf);
        return py::bytes(reinterpret_cast<const char*>(buf.data()), buf.size());
      },
      "Converts an MRPT object to a Python bytes object.");

  m.def(
      "bytesToObject",
      [](const py::bytes& b)
      {
        std::string s = b;
        std::vector<uint8_t> buf(s.begin(), s.end());
        mrpt::serialization::CSerializable::Ptr obj;
        mrpt::serialization::OctetVectorToObject(buf, obj);
        return obj;
      },
      "Converts a Python bytes object back into an MRPT object.");
}