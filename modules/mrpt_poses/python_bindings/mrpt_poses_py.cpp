/* _
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
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

// MRPT headers
#include <mrpt/bayes/CParticleFilterCapable.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose2DInterpolator.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DInterpolator.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPose3DPDFGaussianInf.h>
#include <mrpt/poses/CPose3DPDFParticles.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPosePDFGaussianInf.h>
#include <mrpt/poses/CPosePDFParticles.h>
#include <mrpt/poses/CPoseRandomSampler.h>
#include <mrpt/poses/SO_SE_average.h>
#include <mrpt/serialization/CSerializable.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_MODULE(_bindings, m)
{
  m.doc() = "Python bindings for mrpt_poses";

  // -------------------------------------------------------------------------
  // CPose2D
  // -------------------------------------------------------------------------
  py::class_<
      mrpt::poses::CPose2D, mrpt::serialization::CSerializable,
      std::shared_ptr<mrpt::poses::CPose2D>>(m, "CPose2D")
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
      .def("asTPose", &mrpt::poses::CPose2D::asTPose, "Convert to lightweight mrpt.math.TPose2D")
      .def_static(
          "fromTPose",
          [](const mrpt::math::TPose2D &t) { return mrpt::poses::CPose2D(t.x, t.y, t.phi); },
          "Construct CPose2D from a lightweight mrpt.math.TPose2D")
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
  py::class_<
      mrpt::poses::CPose3D, mrpt::serialization::CSerializable,
      std::shared_ptr<mrpt::poses::CPose3D>>(m, "CPose3D")
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
          "z", [](const mrpt::poses::CPose3D &p) { return p.z(); },
          [](mrpt::poses::CPose3D &p, double val)
          { p.setFromValues(p.x(), p.y(), val, p.yaw(), p.pitch(), p.roll()); },
          "Z coordinate")
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
          "getRotationMatrix", [](const mrpt::poses::CPose3D &p) { return p.getRotationMatrix(); },
          "Returns the 3x3 Rotation Matrix")
      .def(
          "getHomogeneousMatrix",
          [](const mrpt::poses::CPose3D &p) { return p.getHomogeneousMatrix(); },
          "Returns the 4x4 homogeneous transformation matrix")
      .def(
          "getYawPitchRoll", [](const mrpt::poses::CPose3D &p) { return p.getYawPitchRoll(); },
          "Returns (yaw, pitch, roll) as a tuple in radians")
      .def(
          "getInverseHomogeneousMatrix", [](const mrpt::poses::CPose3D &p)
          { return p.getInverseHomogeneousMatrixVal<mrpt::math::CMatrixDouble44>(); })
      .def("setRotationMatrix", &mrpt::poses::CPose3D::setRotationMatrix)
      .def(
          "inverse", [](mrpt::poses::CPose3D &p) { p.inverse(); }, "Inverts the pose in place")
      .def("getOppositeScalar", &mrpt::poses::CPose3D::getOppositeScalar)
      .def("asString", &mrpt::poses::CPose3D::asString)
      .def("asTPose", &mrpt::poses::CPose3D::asTPose, "Convert to lightweight mrpt.math.TPose3D")
      .def_static(
          "fromTPose", [](const mrpt::math::TPose3D &t) { return mrpt::poses::CPose3D(t); },
          "Construct CPose3D from a lightweight mrpt.math.TPose3D")
      .def(
          "composePoint", [](const mrpt::poses::CPose3D &p, const mrpt::math::TPoint3D &pt)
          { return p.composePoint(pt); })
      .def(
          "composePoint",
          [](const mrpt::poses::CPose3D &p, double localX, double localY, double localZ) {
            return p.composePoint({localX, localY, localZ});
          })
      .def(
          "inverseComposePoint", [](const mrpt::poses::CPose3D &p, const mrpt::math::TPoint3D &pt)
          { return p.inverseComposePoint(pt); })
      .def(
          "inverseComposePoint",
          [](const mrpt::poses::CPose3D &p, double globalX, double globalY, double globalZ) {
            return p.inverseComposePoint({globalX, globalY, globalZ});
          })
      // Operators
      .def(py::self + py::self)
      .def(py::self - py::self)
      .def(py::self += py::self)
      .def("__str__", &mrpt::poses::CPose3D::asString)
      .def("__repr__", &mrpt::poses::CPose3D::asString);

  // -------------------------------------------------------------------------
  // PDFs Base Classes
  // -------------------------------------------------------------------------
  py::class_<
      mrpt::poses::CPosePDF, mrpt::serialization::CSerializable,
      std::shared_ptr<mrpt::poses::CPosePDF>>
      cl2DPDF(m, "CPosePDF");
  py::class_<
      mrpt::poses::CPose3DPDF, mrpt::serialization::CSerializable,
      std::shared_ptr<mrpt::poses::CPose3DPDF>>
      cl3DPDF(m, "CPose3DPDF");

  cl2DPDF
      .def(
          "getMean",
          [](const mrpt::poses::CPosePDF &self)
          {
            mrpt::poses::CPose2D mean;
            self.getMean(mean);
            return mean;
          },
          "Returns the mean (expected value) of the distribution")
      .def(
          "getCovarianceAndMean",
          [](const mrpt::poses::CPosePDF &self)
          {
            const auto [cov, mean] = self.getCovarianceAndMean();
            return py::make_tuple(cov, mean);
          },
          "Returns the tuple (cov: CMatrixDouble33, mean: CPose2D)")
      .def(
          "getCovariance", [](const mrpt::poses::CPosePDF &self) { return self.getCovariance(); },
          "Returns the 3x3 covariance matrix")
      .def(
          "saveToTextFile", &mrpt::poses::CPosePDF::saveToTextFile, "file"_a,
          "Saves the distribution to a text file. Returns False on error.")
      .def("__str__", &mrpt::poses::CPosePDF::asString);

  cl3DPDF
      .def(
          "getMean",
          [](const mrpt::poses::CPose3DPDF &self)
          {
            mrpt::poses::CPose3D mean;
            self.getMean(mean);
            return mean;
          },
          "Returns the mean (expected value) of the distribution")
      .def(
          "getCovarianceAndMean",
          [](const mrpt::poses::CPose3DPDF &self)
          {
            const auto [cov, mean] = self.getCovarianceAndMean();
            return py::make_tuple(cov, mean);
          },
          "Returns the tuple (cov: CMatrixDouble66, mean: CPose3D)")
      .def(
          "getCovariance", [](const mrpt::poses::CPose3DPDF &self) { return self.getCovariance(); },
          "Returns the 6x6 covariance matrix")
      .def(
          "saveToTextFile", &mrpt::poses::CPose3DPDF::saveToTextFile, "file"_a,
          "Saves the distribution to a text file. Returns False on error.")
      .def("__str__", &mrpt::poses::CPose3DPDF::asString);

  // -------------------------------------------------------------------------
  // CPosePDFParticles / CPose3DPDFParticles: sample-based pose PDFs
  // -------------------------------------------------------------------------
  py::class_<
      mrpt::poses::CPosePDFParticles, mrpt::poses::CPosePDF, mrpt::bayes::CParticleFilterCapable,
      std::shared_ptr<mrpt::poses::CPosePDFParticles>>(m, "CPosePDFParticles")
      .def(py::init<size_t>(), "M"_a = 1, "Creates M particles at the origin")
      .def("clear", &mrpt::poses::CPosePDFParticles::clear)
      .def("size", &mrpt::poses::CPosePDFParticles::size)
      .def("__len__", &mrpt::poses::CPosePDFParticles::size)
      .def(
          "resetDeterministic", &mrpt::poses::CPosePDFParticles::resetDeterministic, "location"_a,
          "particlesCount"_a = 0,
          "Sets all particles to the given pose (particlesCount=0 keeps the count)")
      .def(
          "resetUniform", &mrpt::poses::CPosePDFParticles::resetUniform, "x_min"_a, "x_max"_a,
          "y_min"_a, "y_max"_a, "phi_min"_a = -M_PI, "phi_max"_a = M_PI, "particlesCount"_a = -1,
          "Spreads particles uniformly in [x_min,x_max]x[y_min,y_max]x[phi_min,phi_max]")
      .def(
          "resetAroundSetOfPoses", &mrpt::poses::CPosePDFParticles::resetAroundSetOfPoses,
          "list_poses"_a, "num_particles_per_pose"_a, "spread_x"_a, "spread_y"_a,
          "spread_phi_rad"_a)
      .def("getParticlePose", &mrpt::poses::CPosePDFParticles::getParticlePose, "i"_a)
      .def("getMostLikelyParticle", &mrpt::poses::CPosePDFParticles::getMostLikelyParticle)
      .def(
          "drawSingleSample",
          [](const mrpt::poses::CPosePDFParticles &self)
          {
            mrpt::poses::CPose2D p;
            self.drawSingleSample(p);
            return p;
          })
      .def(
          "getParticlesAsNumpy",
          [](const mrpt::poses::CPosePDFParticles &self)
          {
            const size_t n = self.size();
            py::array_t<double> arr(std::vector<py::ssize_t>{py::ssize_t(n), 4});
            auto buf = arr.mutable_unchecked<2>();
            for (size_t i = 0; i < n; i++)
            {
              const auto p = self.getParticlePose(i);
              buf(i, 0) = p.x;
              buf(i, 1) = p.y;
              buf(i, 2) = p.phi;
              buf(i, 3) = self.getW(i);
            }
            return arr;
          },
          "Returns all particles as an Nx4 array with columns (x, y, phi, log_weight)")
      .def(
          "__repr__", [](const mrpt::poses::CPosePDFParticles &self)
          { return "CPosePDFParticles(" + std::to_string(self.size()) + " particles)"; });

  py::class_<
      mrpt::poses::CPose3DPDFParticles, mrpt::poses::CPose3DPDF,
      mrpt::bayes::CParticleFilterCapable, std::shared_ptr<mrpt::poses::CPose3DPDFParticles>>(
      m, "CPose3DPDFParticles")
      .def(py::init<size_t>(), "M"_a = 1, "Creates M particles at the origin")
      .def("size", &mrpt::poses::CPose3DPDFParticles::size)
      .def("__len__", &mrpt::poses::CPose3DPDFParticles::size)
      .def(
          "resetDeterministic", &mrpt::poses::CPose3DPDFParticles::resetDeterministic, "location"_a,
          "particlesCount"_a = 0,
          "Sets all particles to the given pose (particlesCount=0 keeps the count)")
      .def(
          "resetUniform", &mrpt::poses::CPose3DPDFParticles::resetUniform, "corner_min"_a,
          "corner_max"_a, "particlesCount"_a = -1,
          "Spreads particles uniformly between two TPose3D corners")
      .def("getParticlePose", &mrpt::poses::CPose3DPDFParticles::getParticlePose, "i"_a)
      .def("getMostLikelyParticle", &mrpt::poses::CPose3DPDFParticles::getMostLikelyParticle)
      .def(
          "drawSingleSample",
          [](const mrpt::poses::CPose3DPDFParticles &self)
          {
            mrpt::poses::CPose3D p;
            self.drawSingleSample(p);
            return p;
          })
      .def(
          "getParticlesAsNumpy",
          [](const mrpt::poses::CPose3DPDFParticles &self)
          {
            const size_t n = self.size();
            py::array_t<double> arr(std::vector<py::ssize_t>{py::ssize_t(n), 7});
            auto buf = arr.mutable_unchecked<2>();
            for (size_t i = 0; i < n; i++)
            {
              const auto p = self.getParticlePose(static_cast<int>(i));
              buf(i, 0) = p.x;
              buf(i, 1) = p.y;
              buf(i, 2) = p.z;
              buf(i, 3) = p.yaw;
              buf(i, 4) = p.pitch;
              buf(i, 5) = p.roll;
              buf(i, 6) = self.getW(i);
            }
            return arr;
          },
          "Returns all particles as an Nx7 array with columns "
          "(x, y, z, yaw, pitch, roll, log_weight)")
      .def(
          "__repr__", [](const mrpt::poses::CPose3DPDFParticles &self)
          { return "CPose3DPDFParticles(" + std::to_string(self.size()) + " particles)"; });

  // -------------------------------------------------------------------------
  // CPose3DPDFGaussian
  // -------------------------------------------------------------------------
  py::class_<
      mrpt::poses::CPose3DPDFGaussian, mrpt::poses::CPose3DPDF,
      std::shared_ptr<mrpt::poses::CPose3DPDFGaussian>>(m, "CPose3DPDFGaussian")
      .def(py::init<>())
      .def(py::init<const mrpt::poses::CPose3D &>())
      .def(py::init<const mrpt::poses::CPose3D &, const mrpt::math::CMatrixDouble66 &>())
      .def_readwrite("mean", &mrpt::poses::CPose3DPDFGaussian::mean)
      .def_readwrite("cov", &mrpt::poses::CPose3DPDFGaussian::cov)
      .def(
          "drawSingleSample",
          [](const mrpt::poses::CPose3DPDFGaussian &self)
          {
            mrpt::poses::CPose3D p;
            self.drawSingleSample(p);
            return p;
          },
          "Draws a single sample from the Gaussian distribution and returns it as a CPose3D.")
      .def("saveToTextFile", &mrpt::poses::CPose3DPDFGaussian::saveToTextFile)
      .def("evaluatePDF", &mrpt::poses::CPose3DPDFGaussian::evaluatePDF)
      .def("evaluateNormalizedPDF", &mrpt::poses::CPose3DPDFGaussian::evaluateNormalizedPDF)
      .def(py::self + py::self)
      .def(py::self += mrpt::poses::CPose3D())
      .def("__str__", &mrpt::poses::CPose3DPDFGaussian::asString);

  // -------------------------------------------------------------------------
  // CPose3DPDFGaussianInf
  // -------------------------------------------------------------------------
  py::class_<
      mrpt::poses::CPose3DPDFGaussianInf, mrpt::poses::CPose3DPDF,
      std::shared_ptr<mrpt::poses::CPose3DPDFGaussianInf>>(m, "CPose3DPDFGaussianInf")
      .def(py::init<>())
      .def(py::init<const mrpt::poses::CPose3D &>())
      .def(
          py::init<const mrpt::poses::CPose3D &, const mrpt::math::CMatrixDouble66 &>(),
          py::arg("mean"), py::arg("inf_matrix"))
      .def_readwrite("mean", &mrpt::poses::CPose3DPDFGaussianInf::mean)
      .def_readwrite("cov_inv", &mrpt::poses::CPose3DPDFGaussianInf::cov_inv)
      .def("isInfType", &mrpt::poses::CPose3DPDFGaussianInf::isInfType)
      .def(
          "drawSingleSample",
          [](const mrpt::poses::CPose3DPDFGaussianInf &self)
          {
            mrpt::poses::CPose3D out;
            self.drawSingleSample(out);
            return out;
          },
          "Draws a single sample from the distribution and returns it as a CPose3D.");

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

  // =========================================================================
  // Phase 0.2 Extensions
  // =========================================================================

  // -------------------------------------------------------------------------
  // CPoint2D
  // -------------------------------------------------------------------------
  py::class_<
      mrpt::poses::CPoint2D, mrpt::serialization::CSerializable,
      std::shared_ptr<mrpt::poses::CPoint2D>>(m, "CPoint2D")
      .def(py::init<>())
      .def(py::init<double, double>(), py::arg("x"), py::arg("y"))
      .def_property(
          "x", [](const mrpt::poses::CPoint2D &p) { return p.x(); },
          [](mrpt::poses::CPoint2D &p, double v) { p.x() = v; })
      .def_property(
          "y", [](const mrpt::poses::CPoint2D &p) { return p.y(); },
          [](mrpt::poses::CPoint2D &p, double v) { p.y() = v; })
      .def("asString", &mrpt::poses::CPoint2D::asString)
      .def(
          "asTPoint", &mrpt::poses::CPoint2D::asTPoint, "Convert to lightweight mrpt.math.TPoint2D")
      .def_static(
          "fromTPoint", [](const mrpt::math::TPoint2D &t) { return mrpt::poses::CPoint2D(t); },
          "Construct CPoint2D from a lightweight mrpt.math.TPoint2D")
      .def("__str__", &mrpt::poses::CPoint2D::asString)
      .def("__repr__", &mrpt::poses::CPoint2D::asString);

  // -------------------------------------------------------------------------
  // CPoint3D
  // -------------------------------------------------------------------------
  py::class_<
      mrpt::poses::CPoint3D, mrpt::serialization::CSerializable,
      std::shared_ptr<mrpt::poses::CPoint3D>>(m, "CPoint3D")
      .def(py::init<>())
      .def(py::init<double, double, double>(), py::arg("x"), py::arg("y"), py::arg("z"))
      .def_property(
          "x", [](const mrpt::poses::CPoint3D &p) { return p.x(); },
          [](mrpt::poses::CPoint3D &p, double v) { p.x() = v; })
      .def_property(
          "y", [](const mrpt::poses::CPoint3D &p) { return p.y(); },
          [](mrpt::poses::CPoint3D &p, double v) { p.y() = v; })
      .def_property(
          "z", [](const mrpt::poses::CPoint3D &p) { return p.z(); },
          [](mrpt::poses::CPoint3D &p, double v) { p.z() = v; })
      .def("asString", &mrpt::poses::CPoint3D::asString)
      .def(
          "asTPoint", &mrpt::poses::CPoint3D::asTPoint, "Convert to lightweight mrpt.math.TPoint3D")
      .def_static(
          "fromTPoint", [](const mrpt::math::TPoint3D &t) { return mrpt::poses::CPoint3D(t); },
          "Construct CPoint3D from a lightweight mrpt.math.TPoint3D")
      .def("__str__", &mrpt::poses::CPoint3D::asString)
      .def("__repr__", &mrpt::poses::CPoint3D::asString);

  // -------------------------------------------------------------------------
  // CPose3DQuat
  // -------------------------------------------------------------------------
  py::class_<
      mrpt::poses::CPose3DQuat, mrpt::serialization::CSerializable,
      std::shared_ptr<mrpt::poses::CPose3DQuat>>(m, "CPose3DQuat")
      .def(py::init<>())
      .def(py::init<const mrpt::poses::CPose3D &>())
      .def_property(
          "x", [](const mrpt::poses::CPose3DQuat &p) { return p.x(); },
          [](mrpt::poses::CPose3DQuat &p, double v) { p.x() = v; })
      .def_property(
          "y", [](const mrpt::poses::CPose3DQuat &p) { return p.y(); },
          [](mrpt::poses::CPose3DQuat &p, double v) { p.y() = v; })
      .def_property(
          "z", [](const mrpt::poses::CPose3DQuat &p) { return p.z(); },
          [](mrpt::poses::CPose3DQuat &p, double v) { p.z() = v; })
      .def_property(
          "quat", py::overload_cast<>(&mrpt::poses::CPose3DQuat::quat),
          [](mrpt::poses::CPose3DQuat &p, const mrpt::math::CQuaternionDouble &q) { p.quat() = q; },
          py::return_value_policy::reference_internal)
      .def("norm", &mrpt::poses::CPose3DQuat::norm)
      .def("asString", &mrpt::poses::CPose3DQuat::asString)
      .def(
          "asTPose", &mrpt::poses::CPose3DQuat::asTPose,
          "Convert to lightweight mrpt.math.TPose3DQuat")
      .def_static(
          "fromTPose",
          [](const mrpt::math::TPose3DQuat &t)
          {
            mrpt::poses::CPose3DQuat ret;
            ret.x() = t.x;
            ret.y() = t.y;
            ret.z() = t.z;
            ret.quat().r(t.qr);
            ret.quat().x(t.qx);
            ret.quat().y(t.qy);
            ret.quat().z(t.qz);
            return ret;
          },
          "Construct CPose3DQuat from a lightweight mrpt.math.TPose3DQuat")
      .def("__str__", &mrpt::poses::CPose3DQuat::asString)
      .def("__repr__", &mrpt::poses::CPose3DQuat::asString);

  // -------------------------------------------------------------------------
  // CPosePDFGaussian
  // -------------------------------------------------------------------------
  py::class_<
      mrpt::poses::CPosePDFGaussian, mrpt::poses::CPosePDF,
      std::shared_ptr<mrpt::poses::CPosePDFGaussian>>(m, "CPosePDFGaussian")
      .def(py::init<>())
      .def(py::init<const mrpt::poses::CPose2D &>())
      .def(py::init<const mrpt::poses::CPose2D &, const mrpt::math::CMatrixDouble33 &>())
      .def_readwrite("mean", &mrpt::poses::CPosePDFGaussian::mean)
      .def_readwrite("cov", &mrpt::poses::CPosePDFGaussian::cov)
      .def(
          "drawSingleSample",
          [](const mrpt::poses::CPosePDFGaussian &self)
          {
            mrpt::poses::CPose2D out;
            self.drawSingleSample(out);
            return out;
          },
          "Draw a single sample from the Gaussian distribution")
      .def("evaluatePDF", &mrpt::poses::CPosePDFGaussian::evaluatePDF)
      .def("evaluateNormalizedPDF", &mrpt::poses::CPosePDFGaussian::evaluateNormalizedPDF)
      .def("__str__", &mrpt::poses::CPosePDFGaussian::asString)
      .def("__repr__", &mrpt::poses::CPosePDFGaussian::asString);

  // -------------------------------------------------------------------------
  // CPosePDFGaussianInf
  // -------------------------------------------------------------------------
  py::class_<
      mrpt::poses::CPosePDFGaussianInf, mrpt::poses::CPosePDF,
      std::shared_ptr<mrpt::poses::CPosePDFGaussianInf>>(m, "CPosePDFGaussianInf")
      .def(py::init<>())
      .def(py::init<const mrpt::poses::CPose2D &>())
      .def(
          py::init<const mrpt::poses::CPose2D &, const mrpt::math::CMatrixDouble33 &>(),
          py::arg("mean"), py::arg("inf_matrix"))
      .def_readwrite("mean", &mrpt::poses::CPosePDFGaussianInf::mean)
      .def_readwrite("cov_inv", &mrpt::poses::CPosePDFGaussianInf::cov_inv)
      .def(
          "drawSingleSample",
          [](const mrpt::poses::CPosePDFGaussianInf &self)
          {
            mrpt::poses::CPose2D out;
            self.drawSingleSample(out);
            return out;
          })
      .def("__str__", &mrpt::poses::CPosePDFGaussianInf::asString)
      .def("__repr__", &mrpt::poses::CPosePDFGaussianInf::asString);

  // -------------------------------------------------------------------------
  // CPose2DInterpolator
  // -------------------------------------------------------------------------
  py::class_<
      mrpt::poses::CPose2DInterpolator, mrpt::serialization::CSerializable,
      std::shared_ptr<mrpt::poses::CPose2DInterpolator>>(m, "CPose2DInterpolator")
      .def(py::init<>())
      .def(
          "insert", py::overload_cast<const mrpt::Clock::time_point &, const mrpt::math::TPose2D &>(
                        &mrpt::poses::CPose2DInterpolator::insert))
      .def(
          "interpolate",
          [](const mrpt::poses::CPose2DInterpolator &self, const mrpt::Clock::time_point &t)
          {
            mrpt::math::TPose2D out;
            bool valid = false;
            self.interpolate(t, out, valid);
            return py::make_tuple(out, valid);
          },
          "Returns (TPose2D, valid) — interpolated pose at given time")
      .def("size", &mrpt::poses::CPose2DInterpolator::size)
      .def("empty", &mrpt::poses::CPose2DInterpolator::empty)
      .def("clear", &mrpt::poses::CPose2DInterpolator::clear)
      .def("__len__", [](const mrpt::poses::CPose2DInterpolator &self) { return self.size(); });

  // -------------------------------------------------------------------------
  // CPose3DInterpolator
  // -------------------------------------------------------------------------
  py::class_<
      mrpt::poses::CPose3DInterpolator, mrpt::serialization::CSerializable,
      std::shared_ptr<mrpt::poses::CPose3DInterpolator>>(m, "CPose3DInterpolator")
      .def(py::init<>())
      .def(
          "insert", py::overload_cast<const mrpt::Clock::time_point &, const mrpt::math::TPose3D &>(
                        &mrpt::poses::CPose3DInterpolator::insert))
      .def(
          "interpolate",
          [](const mrpt::poses::CPose3DInterpolator &self, const mrpt::Clock::time_point &t)
          {
            mrpt::math::TPose3D out;
            bool valid = false;
            self.interpolate(t, out, valid);
            return py::make_tuple(out, valid);
          },
          "Returns (TPose3D, valid) — interpolated pose at given time")
      .def("size", &mrpt::poses::CPose3DInterpolator::size)
      .def("empty", &mrpt::poses::CPose3DInterpolator::empty)
      .def("clear", &mrpt::poses::CPose3DInterpolator::clear)
      .def("__len__", [](const mrpt::poses::CPose3DInterpolator &self) { return self.size(); });

  // -------------------------------------------------------------------------
  // CPoseRandomSampler
  // -------------------------------------------------------------------------
  py::class_<mrpt::poses::CPoseRandomSampler, std::shared_ptr<mrpt::poses::CPoseRandomSampler>>(
      m, "CPoseRandomSampler")
      .def(py::init<>())
      .def(
          "setPosePDF",
          [](mrpt::poses::CPoseRandomSampler &self,
             const std::shared_ptr<mrpt::poses::CPosePDF> &pdf) { self.setPosePDF(*pdf); })
      .def(
          "setPosePDF",
          [](mrpt::poses::CPoseRandomSampler &self,
             const std::shared_ptr<mrpt::poses::CPose3DPDF> &pdf) { self.setPosePDF(*pdf); })
      .def(
          "drawSample2D",
          [](const mrpt::poses::CPoseRandomSampler &self)
          {
            mrpt::poses::CPose2D out;
            self.drawSample(out);
            return out;
          },
          "Draw a 2D pose sample")
      .def(
          "drawSample3D",
          [](const mrpt::poses::CPoseRandomSampler &self)
          {
            mrpt::poses::CPose3D out;
            self.drawSample(out);
            return out;
          },
          "Draw a 3D pose sample");
}