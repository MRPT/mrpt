/* _
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
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

// MRPT headers
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPose3DPDFGaussianInf.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/SO_SE_average.h>
#include <mrpt/serialization/CSerializable.h>

namespace py = pybind11;

PYBIND11_MODULE(_bindings, m)
{
  m.doc() = "Python bindings for mrpt_poses";

  // -------------------------------------------------------------------------
  // CPose2D
  // -------------------------------------------------------------------------
  py::class_<mrpt::poses::CPose2D, mrpt::serialization::CSerializable>(m, "CPose2D")
      .def(py::init<>(), "Default constructor (0,0,0).")
      .def(
          py::init<double, double, double>(), py::arg("x"), py::arg("y"), py::arg("phi"),
          "Constructor from coordinates.")
      .def(
          py::init<const mrpt::poses::CPose3D &>(),
          "Construct from CPose3D (loss of z/pitch/roll).")
      // Properties (Getter/Setter wrappers)
      .def_property(
          "x", [](const mrpt::poses::CPose2D &p) { return p.x(); },
          [](mrpt::poses::CPose2D &p, double val) { p.x(val); }, "X coordinate")
      .def_property(
          "y", [](const mrpt::poses::CPose2D &p) { return p.y(); },
          [](mrpt::poses::CPose2D &p, double val) { p.y(val); }, "Y coordinate")
      .def_property(
          "phi", [](const mrpt::poses::CPose2D &p) { return p.phi(); },
          [](mrpt::poses::CPose2D &p, double val) { p.phi(val); }, "Phi orientation (radians)")
      // Methods
      .def("normalizePhi", &mrpt::poses::CPose2D::normalizePhi, "Forces phi to be in [-pi,pi]")
      .def("asTPose", &mrpt::poses::CPose2D::asTPose, "Returns lightweight struct")
      .def("norm", &mrpt::poses::CPose2D::norm, "Returns the norm of the (x,y) vector")
      .def("asString", &mrpt::poses::CPose2D::asString, "Returns human-readable string [x y phi]")
      .def("fromString", &mrpt::poses::CPose2D::fromString, "Set value from string")
      .def(
          "inverse",
          [](const mrpt::poses::CPose2D &p)
          {
            mrpt::poses::CPose2D ret = p;
            ret.inverse();
            return ret;
          },
          "Returns the inverse pose")
      // Operators
      .def(py::self + py::self)
      .def(py::self - py::self)
      .def(py::self += py::self)
      .def("__str__", &mrpt::poses::CPose2D::asString)
      .def("__repr__", &mrpt::poses::CPose2D::asString);

  // -------------------------------------------------------------------------
  // CPose3D
  // -------------------------------------------------------------------------
  py::class_<mrpt::poses::CPose3D, mrpt::serialization::CSerializable>(m, "CPose3D")
      .def(py::init<>())
      .def(
          py::init<double, double, double, double, double, double>(), py::arg("x"), py::arg("y"),
          py::arg("z"), py::arg("yaw") = 0, py::arg("pitch") = 0, py::arg("roll") = 0)
      .def(py::init<const mrpt::poses::CPose2D &>())
      // Static Builders
      .def_static("FromXYZYawPitchRoll", &mrpt::poses::CPose3D::FromXYZYawPitchRoll)
      .def_static("FromYawPitchRoll", &mrpt::poses::CPose3D::FromYawPitchRoll)
      .def_static(
          "FromTranslation",
          py::overload_cast<double, double, double>(&mrpt::poses::CPose3D::FromTranslation))
      // Properties
      .def_property(
          "x", [](const mrpt::poses::CPose3D &p) { return p.x(); },
          [](mrpt::poses::CPose3D &p, double val) { p.x(val); }, "X coordinate")
      .def_property(
          "y", [](const mrpt::poses::CPose3D &p) { return p.y(); },
          [](mrpt::poses::CPose3D &p, double val) { p.y(val); }, "Y coordinate")
      .def_property(
          "z",
          [](const mrpt::poses::CPose3D &p)
          { return p.m_coords[2]; },  // Accessing z via m_coords directly or logic
          [](mrpt::poses::CPose3D &p, double val) { p.m_coords[2] = val; }, "Z coordinate")
      .def_property(
          "yaw", &mrpt::poses::CPose3D::yaw,
          [](mrpt::poses::CPose3D &p, double val) { p.setYawPitchRoll(val, p.pitch(), p.roll()); })
      .def_property(
          "pitch", &mrpt::poses::CPose3D::pitch,
          [](mrpt::poses::CPose3D &p, double val) { p.setYawPitchRoll(p.yaw(), val, p.roll()); })
      .def_property(
          "roll", &mrpt::poses::CPose3D::roll,
          [](mrpt::poses::CPose3D &p, double val) { p.setYawPitchRoll(p.yaw(), p.pitch(), val); })
      // Methods
      .def("setYawPitchRoll", &mrpt::poses::CPose3D::setYawPitchRoll)
      .def("setFromValues", &mrpt::poses::CPose3D::setFromValues)
      .def(
          "getRotationMatrix",
          [](const mrpt::poses::CPose3D &p)
          {
            mrpt::math::CMatrixDouble33 R;
            p.getRotationMatrix(R);
            return R;
          },
          "Returns the 3x3 Rotation Matrix")
      .def("setRotationMatrix", &mrpt::poses::CPose3D::setRotationMatrix)
      .def(
          "inverse", [](mrpt::poses::CPose3D &p) { p.inverse(); }, "Inverts the pose in place")
      .def("getOppositeScalar", &mrpt::poses::CPose3D::getOppositeScalar)
      .def("asString", &mrpt::poses::CPose3D::asString)
      // Operators
      .def(py::self + py::self)
      .def(py::self - py::self)
      .def(py::self += py::self)
      .def("__str__", &mrpt::poses::CPose3D::asString)
      .def("__repr__", &mrpt::poses::CPose3D::asString);

  // -------------------------------------------------------------------------
  // PDFs Base Classes
  // -------------------------------------------------------------------------
  py::class_<mrpt::poses::CPosePDF, mrpt::serialization::CSerializable> cl2DPDF(m, "CPosePDF");
  py::class_<mrpt::poses::CPose3DPDF, mrpt::serialization::CSerializable> cl3DPDF(m, "CPose3DPDF");

  // -------------------------------------------------------------------------
  // CPose3DPDFGaussian
  // -------------------------------------------------------------------------
  py::class_<mrpt::poses::CPose3DPDFGaussian, mrpt::poses::CPose3DPDF>(m, "CPose3DPDFGaussian")
      .def(py::init<>())
      .def(py::init<const mrpt::poses::CPose3D &>())
      .def(py::init<const mrpt::poses::CPose3D &, const mrpt::math::CMatrixDouble66 &>())
      .def_readwrite("mean", &mrpt::poses::CPose3DPDFGaussian::mean)
      .def_readwrite("cov", &mrpt::poses::CPose3DPDFGaussian::cov)
      .def("drawSingleSample", &mrpt::poses::CPose3DPDFGaussian::drawSingleSample)
      .def("saveToTextFile", &mrpt::poses::CPose3DPDFGaussian::saveToTextFile)
      .def("evaluatePDF", &mrpt::poses::CPose3DPDFGaussian::evaluatePDF)
      .def("evaluateNormalizedPDF", &mrpt::poses::CPose3DPDFGaussian::evaluateNormalizedPDF)
      .def(py::self + py::self)
      .def(py::self += mrpt::poses::CPose3D())
      .def("__str__", &mrpt::poses::CPose3DPDFGaussian::asString);

  // -------------------------------------------------------------------------
  // CPose3DPDFGaussianInf
  // -------------------------------------------------------------------------
  py::class_<mrpt::poses::CPose3DPDFGaussianInf, mrpt::poses::CPose3DPDF>(
      m, "CPose3DPDFGaussianInf")
      .def(py::init<>())
      .def(py::init<const mrpt::poses::CPose3D &>())
      .def(
          py::init<const mrpt::poses::CPose3D &, const mrpt::math::CMatrixDouble66 &>(),
          py::arg("mean"), py::arg("inf_matrix"))
      .def_readwrite("mean", &mrpt::poses::CPose3DPDFGaussianInf::mean)
      .def_readwrite("cov_inv", &mrpt::poses::CPose3DPDFGaussianInf::cov_inv)
      .def("isInfType", &mrpt::poses::CPose3DPDFGaussianInf::isInfType)
      .def("drawSingleSample", &mrpt::poses::CPose3DPDFGaussianInf::drawSingleSample);

  // -------------------------------------------------------------------------
  // Averaging SE(2) and SE(3)
  // -------------------------------------------------------------------------
  py::class_<mrpt::poses::SE_average<2>>(m, "SE_average2")
      .def(py::init<>())
      .def("clear", &mrpt::poses::SE_average<2>::clear)
      .def(
          "append",
          py::overload_cast<const mrpt::poses::CPose2D &>(&mrpt::poses::SE_average<2>::append))
      .def(
          "append", py::overload_cast<const mrpt::poses::CPose2D &, const double>(
                        &mrpt::poses::SE_average<2>::append))
      .def(
          "get_average",
          [](const mrpt::poses::SE_average<2> &self)
          {
            mrpt::poses::CPose2D out;
            self.get_average(out);
            return out;
          },
          "Returns the calculated average pose.");

  py::class_<mrpt::poses::SE_average<3>>(m, "SE_average3")
      .def(py::init<>())
      .def("clear", &mrpt::poses::SE_average<3>::clear)
      .def(
          "append",
          py::overload_cast<const mrpt::poses::CPose3D &>(&mrpt::poses::SE_average<3>::append))
      .def(
          "append", py::overload_cast<const mrpt::poses::CPose3D &, const double>(
                        &mrpt::poses::SE_average<3>::append))
      .def(
          "get_average",
          [](const mrpt::poses::SE_average<3> &self)
          {
            mrpt::poses::CPose3D out;
            self.get_average(out);
            return out;
          },
          "Returns the calculated average pose.");
}