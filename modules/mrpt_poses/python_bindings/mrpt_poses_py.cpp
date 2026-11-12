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
#include <mrpt/poses/CPosePDFSOG.h>
#include <mrpt/poses/CPoseRandomSampler.h>
#include <mrpt/poses/SO_SE_average.h>
#include <mrpt/serialization/CSerializable.h>

namespace py = pybind11;
using namespace pybind11::literals;

namespace
{
// The Python class already registered for T, to add more methods to it:
template <class T>
py::class_<T, std::shared_ptr<T>> registeredClass()
{
  return py::reinterpret_borrow<py::class_<T, std::shared_ptr<T>>>(py::type::of<T>());
}

// Copy constructor, __copy__ and __deepcopy__ (these classes hold no shared data).
template <class T>
void addCopyMethods()
{
  registeredClass<T>()
      .def(py::init<const T &>(), "other"_a, "Copy constructor.")
      .def(
          "__copy__", [](const T &self) { return T(self); }, "Returns a copy of this object.")
      .def(
          "__deepcopy__", [](const T &self, const py::dict & /*memo*/) { return T(self); },
          "memo"_a, "Returns a copy of this object.");
}
}  // namespace

PYBIND11_MODULE(_bindings, m)
{
  m.doc() = "Python bindings for mrpt_poses";

  // Registered before its first use as an argument type (CPose2D ctor), so
  // that signatures and stubs name it:
  py::class_<
      mrpt::poses::CPose3D, mrpt::serialization::CSerializable,
      std::shared_ptr<mrpt::poses::CPose3D>>
      pose3D(
          m, "CPose3D",
          "SE(3) rigid-body pose (x, y, z, yaw, pitch, roll), with a cached rotation matrix.");

  // -------------------------------------------------------------------------
  // CPose2D
  // -------------------------------------------------------------------------
  py::class_<
      mrpt::poses::CPose2D, mrpt::serialization::CSerializable,
      std::shared_ptr<mrpt::poses::CPose2D>>(m, "CPose2D", "SE(2) rigid-body pose (x, y, phi).")
      .def(py::init<>(), "Default constructor (0,0,0).")
      .def(
          py::init<double, double, double>(), py::arg("x"), py::arg("y"), py::arg("phi"),
          "Constructor from coordinates.")
      .def(
          py::init<const mrpt::poses::CPose3D &>(), py::arg("p"),
          "Construct from CPose3D (loss of z/pitch/roll).")
      .def(
          py::init<const mrpt::math::TPose2D &>(), py::arg("p"),
          "Construct from a lightweight mrpt.math.TPose2D.")
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
          "Returns the inverse pose as a new object (same as -p), without modifying this one. "
          "Note that CPose3D.inverse() works in place instead.")
      .def(
          "composePoint",
          [](const mrpt::poses::CPose2D &p, const mrpt::math::TPoint2D &l)
          {
            mrpt::math::TPoint2D g;
            p.composePoint(l, g);
            return g;
          },
          "l"_a, "Transforms a point from the local frame of this pose into the global frame.")
      .def(
          "composePoint",
          [](const mrpt::poses::CPose2D &p, double lx, double ly)
          {
            mrpt::math::TPoint2D g;
            p.composePoint(lx, ly, g.x, g.y);
            return g;
          },
          "lx"_a, "ly"_a,
          "Transforms a point (x, y) from the local frame of this pose into the global frame.")
      .def(
          "composePoint",
          [](const mrpt::poses::CPose2D &p, const mrpt::math::TPoint3D &l)
          { return p.composePoint(l); },
          "l"_a,
          "Transforms a 3D point from the local frame of this pose into the global frame (z is "
          "not modified).")
      .def(
          "inverseComposePoint",
          [](const mrpt::poses::CPose2D &p, const mrpt::math::TPoint2D &g)
          { return p.inverseComposePoint(g); },
          "g"_a, "Transforms a point from the global frame into the local frame of this pose.")
      .def(
          "inverseComposePoint",
          [](const mrpt::poses::CPose2D &p, double gx, double gy)
          {
            mrpt::math::TPoint2D l;
            p.inverseComposePoint(gx, gy, l.x, l.y);
            return l;
          },
          "gx"_a, "gy"_a,
          "Transforms a point (x, y) from the global frame into the local frame of this pose.")
      // Operators
      .def(py::self + py::self)
      .def(py::self - py::self)
      .def(py::self += py::self)
      .def(
          -py::self,
          "Returns the inverse pose (SE(2) group inverse, not the negation of each coordinate).")
      .def("__str__", &mrpt::poses::CPose2D::asString)
      .def("__repr__", &mrpt::poses::CPose2D::asString);

  // -------------------------------------------------------------------------
  // CPose3D
  // -------------------------------------------------------------------------
  pose3D.def(py::init<>(), "Default constructor, with all the coordinates set to zero.")
      .def(
          py::init<double, double, double, double, double, double>(), py::arg("x"), py::arg("y"),
          py::arg("z"), py::arg("yaw") = 0, py::arg("pitch") = 0, py::arg("roll") = 0,
          "Constructor with Initialization of the pose, translation (x,y,z) in meters, "
          "(yaw,pitch,roll) angles in radians.")
      .def(
          py::init<const mrpt::poses::CPose2D &>(),
          "Builds a 3D pose from a 2D pose (z, pitch and roll set to zero).")
      .def(
          py::init<const mrpt::math::TPose3D &>(), py::arg("p"),
          "Builds the pose from a lightweight mrpt.math.TPose3D.")
      // Static Builders
      .def_static(
          "FromXYZYawPitchRoll", &mrpt::poses::CPose3D::FromXYZYawPitchRoll,
          "Builds a pose from a translation (x,y,z) in meters and (yaw,pitch,roll) angles in "
          "radians.")
      .def_static(
          "FromYawPitchRoll", &mrpt::poses::CPose3D::FromYawPitchRoll,
          "Builds a pose with a null translation and (yaw,pitch,roll) angles in radians.")
      .def_static(
          "FromTranslation",
          py::overload_cast<double, double, double>(&mrpt::poses::CPose3D::FromTranslation),
          "Builds a pose with a translation without rotation.")
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
      .def(
          "setYawPitchRoll", &mrpt::poses::CPose3D::setYawPitchRoll,
          "Sets the three rotation angles, in radians.")
      .def(
          "setFromValues", &mrpt::poses::CPose3D::setFromValues,
          "Sets the pose from a position (meters) and yaw, pitch, roll angles (radians).")
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
          "getInverseHomogeneousMatrix",
          [](const mrpt::poses::CPose3D &p)
          { return p.getInverseHomogeneousMatrixVal<mrpt::math::CMatrixDouble44>(); },
          "Returns the corresponding 4x4 inverse homogeneous transformation matrix for this point "
          "or pose.")
      .def(
          "setRotationMatrix", &mrpt::poses::CPose3D::setRotationMatrix,
          "Sets the 3x3 rotation matrix.")
      .def(
          "inverse", [](mrpt::poses::CPose3D &p) { p.inverse(); },
          "Inverts this pose in place and returns None. Use -p to get the inverse as a new pose "
          "instead (note that CPose2D.inverse() returns a new pose).")
      .def(
          "getOppositeScalar", &mrpt::poses::CPose3D::getOppositeScalar,
          "Return the opposite of the current pose instance by taking the negative of all its "
          "components individually.")
      .def(
          "asString", &mrpt::poses::CPose3D::asString,
          "Returns a human-readable textual representation of the object (eg: \"[x y z yaw pitch "
          "roll]\", angles in degrees.)")
      .def("asTPose", &mrpt::poses::CPose3D::asTPose, "Convert to lightweight mrpt.math.TPose3D")
      .def_static(
          "fromTPose", [](const mrpt::math::TPose3D &t) { return mrpt::poses::CPose3D(t); },
          "Construct CPose3D from a lightweight mrpt.math.TPose3D")
      .def(
          "composePoint",
          [](const mrpt::poses::CPose3D &p, const mrpt::math::TPoint3D &pt)
          { return p.composePoint(pt); },
          "Transforms a point from the local frame of this pose into the global frame.")
      .def(
          "composePoint",
          [](const mrpt::poses::CPose3D &p, double localX, double localY, double localZ) {
            return p.composePoint({localX, localY, localZ});
          },
          "Transforms a point (x, y, z) from the local frame of this pose into the global frame.")
      .def(
          "inverseComposePoint",
          [](const mrpt::poses::CPose3D &p, const mrpt::math::TPoint3D &pt)
          { return p.inverseComposePoint(pt); },
          "Transforms a point from the global frame into the local frame of this pose.")
      .def(
          "inverseComposePoint",
          [](const mrpt::poses::CPose3D &p, double globalX, double globalY, double globalZ) {
            return p.inverseComposePoint({globalX, globalY, globalZ});
          },
          "Transforms a point (x, y, z) from the global frame into the local frame of this pose.")
      // Operators
      .def(py::self + py::self)
      .def(py::self - py::self)
      .def(py::self += py::self)
      .def(
          -py::self,
          "Returns the inverse pose as a new object (SE(3) group inverse, not the negation of "
          "each coordinate). This pose is not modified.")
      .def("__str__", &mrpt::poses::CPose3D::asString)
      .def("__repr__", &mrpt::poses::CPose3D::asString);

  // -------------------------------------------------------------------------
  // PDFs Base Classes
  // -------------------------------------------------------------------------
  py::class_<
      mrpt::poses::CPosePDF, mrpt::serialization::CSerializable,
      std::shared_ptr<mrpt::poses::CPosePDF>>
      cl2DPDF(
          m, "CPosePDF",
          "Base class of probability density functions (PDFs) of a 2D pose (x, y, phi).");
  py::class_<
      mrpt::poses::CPose3DPDF, mrpt::serialization::CSerializable,
      std::shared_ptr<mrpt::poses::CPose3DPDF>>
      cl3DPDF(m, "CPose3DPDF", "Base class of probability density functions (PDFs) of a 3D pose.");

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
      std::shared_ptr<mrpt::poses::CPosePDFParticles>>(
      m, "CPosePDFParticles", "A PDF of a 2D pose, as a set of weighted samples (particles).")
      .def(py::init<size_t>(), "M"_a = 1, "Creates M particles at the origin")
      .def("clear", &mrpt::poses::CPosePDFParticles::clear, "Removes all particles.")
      .def("size", &mrpt::poses::CPosePDFParticles::size, "Returns the number of particles.")
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
          "spread_phi_rad"_a,
          "Resets the particles around a set of poses (x, y, phi), with a given number of "
          "particles per pose and spread.")
      .def(
          "getParticlePose", &mrpt::poses::CPosePDFParticles::getParticlePose, "i"_a,
          "Returns the pose of the i'th particle.")
      .def(
          "getMostLikelyParticle", &mrpt::poses::CPosePDFParticles::getMostLikelyParticle,
          "Returns the particle with the highest weight.")
      .def(
          "drawSingleSample",
          [](const mrpt::poses::CPosePDFParticles &self)
          {
            mrpt::poses::CPose2D p;
            self.drawSingleSample(p);
            return p;
          },
          "Draws one sample from the distribution (weights must be normalized).")
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
      m, "CPose3DPDFParticles", "A PDF of a 3D pose, as a set of weighted samples (particles).")
      .def(py::init<size_t>(), "M"_a = 1, "Creates M particles at the origin")
      .def("size", &mrpt::poses::CPose3DPDFParticles::size, "Returns the number of particles.")
      .def("__len__", &mrpt::poses::CPose3DPDFParticles::size)
      .def(
          "resetDeterministic", &mrpt::poses::CPose3DPDFParticles::resetDeterministic, "location"_a,
          "particlesCount"_a = 0,
          "Sets all particles to the given pose (particlesCount=0 keeps the count)")
      .def(
          "resetUniform", &mrpt::poses::CPose3DPDFParticles::resetUniform, "corner_min"_a,
          "corner_max"_a, "particlesCount"_a = -1,
          "Spreads particles uniformly between two TPose3D corners")
      .def(
          "getParticlePose", &mrpt::poses::CPose3DPDFParticles::getParticlePose, "i"_a,
          "Returns the pose of the i'th particle.")
      .def(
          "getMostLikelyParticle", &mrpt::poses::CPose3DPDFParticles::getMostLikelyParticle,
          "Returns the particle with the highest weight.")
      .def(
          "drawSingleSample",
          [](const mrpt::poses::CPose3DPDFParticles &self)
          {
            mrpt::poses::CPose3D p;
            self.drawSingleSample(p);
            return p;
          },
          "Draws one sample from the distribution (weights must be normalized).")
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
      std::shared_ptr<mrpt::poses::CPose3DPDFGaussian>>(
      m, "CPose3DPDFGaussian",
      "A PDF of a 3D pose as a Gaussian with a mean and a 6x6 covariance matrix.")
      .def(py::init<>(), "Default constructor.")
      .def(
          py::init<const mrpt::poses::CPose3D &>(),
          "Builds the PDF from a mean, with zero covariance.")
      .def(
          py::init<const mrpt::poses::CPose3D &, const mrpt::math::CMatrixDouble66 &>(),
          "Builds the PDF from a mean and a 6x6 covariance matrix.")
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
      .def(
          "saveToTextFile", &mrpt::poses::CPose3DPDFGaussian::saveToTextFile,
          "Saves the mean and covariance to a text file.")
      .def(
          "evaluatePDF", &mrpt::poses::CPose3DPDFGaussian::evaluatePDF,
          "Evaluates the PDF at a given point.")
      .def(
          "evaluateNormalizedPDF", &mrpt::poses::CPose3DPDFGaussian::evaluateNormalizedPDF,
          "Evaluates the ratio PDF(x) / PDF(MEAN), that is, the normalized PDF in the range [0,1].")
      .def(py::self + py::self)
      .def(
          py::self - py::self,
          "Inverse pose composition a (-) b, propagating both covariances (independent "
          "variables).")
      .def(-py::self, "Returns the PDF of the inverse pose, (0,0,0) (-) this.")
      // Lambdas, since the C++ in-place operators return void:
      .def(
          "__iadd__",
          [](mrpt::poses::CPose3DPDFGaussian &self,
             const mrpt::poses::CPose3D &Ap) -> mrpt::poses::CPose3DPDFGaussian &
          {
            self += Ap;
            return self;
          },
          py::is_operator(), "Makes this = this (+) Ap, for a pose without uncertainty.")
      .def(
          "__iadd__",
          [](mrpt::poses::CPose3DPDFGaussian &self,
             const mrpt::poses::CPose3DPDFGaussian &Ap) -> mrpt::poses::CPose3DPDFGaussian &
          {
            self += Ap;
            return self;
          },
          py::is_operator(), "Makes this = this (+) Ap, propagating both covariances.")
      .def(
          "__isub__",
          [](mrpt::poses::CPose3DPDFGaussian &self,
             const mrpt::poses::CPose3DPDFGaussian &Ap) -> mrpt::poses::CPose3DPDFGaussian &
          {
            self -= Ap;
            return self;
          },
          py::is_operator(), "Makes this = this (-) Ap, propagating both covariances.")
      .def(
          "inverse",
          [](const mrpt::poses::CPose3DPDFGaussian &self)
          {
            mrpt::poses::CPose3DPDFGaussian out;
            self.inverse(out);
            return out;
          },
          "Returns the PDF of the inverse pose, (0,0,0) (-) this, as a new object (same as "
          "-pdf). This PDF is not modified.")
      .def(
          "changeCoordinatesReference",
          &mrpt::poses::CPose3DPDFGaussian::changeCoordinatesReference, "newReferenceBase"_a,
          "Makes this = newReferenceBase (+) this, for a reference pose without uncertainty.")
      .def("__str__", &mrpt::poses::CPose3DPDFGaussian::asString);

  // -------------------------------------------------------------------------
  // CPose3DPDFGaussianInf
  // -------------------------------------------------------------------------
  py::class_<
      mrpt::poses::CPose3DPDFGaussianInf, mrpt::poses::CPose3DPDF,
      std::shared_ptr<mrpt::poses::CPose3DPDFGaussianInf>>(
      m, "CPose3DPDFGaussianInf",
      "A PDF of a 3D pose as a Gaussian with a mean and a 6x6 information (inverse covariance) "
      "matrix.")
      .def(py::init<>(), "Default constructor: zero mean and zero information matrix.")
      .def(
          py::init<const mrpt::poses::CPose3D &>(),
          "Builds the PDF from a mean, with zero information matrix.")
      .def(
          py::init<const mrpt::poses::CPose3D &, const mrpt::math::CMatrixDouble66 &>(),
          py::arg("mean"), py::arg("inf_matrix"),
          "Builds the PDF from a mean and a 6x6 information matrix.")
      .def_readwrite("mean", &mrpt::poses::CPose3DPDFGaussianInf::mean)
      .def_readwrite("cov_inv", &mrpt::poses::CPose3DPDFGaussianInf::cov_inv)
      .def(
          "isInfType", &mrpt::poses::CPose3DPDFGaussianInf::isInfType,
          "Returns whether the class instance holds the uncertainty in covariance or information "
          "form.")
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
  py::class_<mrpt::poses::SE_average<2>>(
      m, "SE_average2", "Computes the (optionally weighted) average of a set of SE(2) poses.")
      .def(py::init<>(), "Default constructor.")
      .def("clear", &mrpt::poses::SE_average<2>::clear, "Resets the accumulated poses.")
      .def(
          "append",
          py::overload_cast<const mrpt::poses::CPose2D &>(&mrpt::poses::SE_average<2>::append),
          "Adds a pose with unit weight.")
      .def(
          "append",
          py::overload_cast<const mrpt::poses::CPose2D &, const double>(
              &mrpt::poses::SE_average<2>::append),
          "Adds a pose with the given weight.")
      .def(
          "get_average",
          [](const mrpt::poses::SE_average<2> &self)
          {
            mrpt::poses::CPose2D out;
            self.get_average(out);
            return out;
          },
          "Returns the calculated average pose.");

  py::class_<mrpt::poses::SE_average<3>>(
      m, "SE_average3", "Computes the (optionally weighted) average of a set of SE(3) poses.")
      .def(py::init<>(), "Default constructor.")
      .def("clear", &mrpt::poses::SE_average<3>::clear, "Resets the accumulated poses.")
      .def(
          "append",
          py::overload_cast<const mrpt::poses::CPose3D &>(&mrpt::poses::SE_average<3>::append),
          "Adds a pose with unit weight.")
      .def(
          "append",
          py::overload_cast<const mrpt::poses::CPose3D &, const double>(
              &mrpt::poses::SE_average<3>::append),
          "Adds a pose with the given weight.")
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
      std::shared_ptr<mrpt::poses::CPoint2D>>(m, "CPoint2D", "A class used to store a 2D point.")
      .def(py::init<>(), "Default constructor.")
      .def(
          py::init<double, double>(), py::arg("x"), py::arg("y"),
          "Constructor for initializing point coordinates.")
      .def_property(
          "x", [](const mrpt::poses::CPoint2D &p) { return p.x(); },
          [](mrpt::poses::CPoint2D &p, double v) { p.x() = v; })
      .def_property(
          "y", [](const mrpt::poses::CPoint2D &p) { return p.y(); },
          [](mrpt::poses::CPoint2D &p, double v) { p.y() = v; })
      .def(
          "asString", &mrpt::poses::CPoint2D::asString,
          "Returns a text representation, e.g. \"[0.02 1.04]\".")
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
      std::shared_ptr<mrpt::poses::CPoint3D>>(m, "CPoint3D", "A class used to store a 3D point.")
      .def(py::init<>(), "Default constructor.")
      .def(
          py::init<double, double, double>(), py::arg("x"), py::arg("y"), py::arg("z"),
          "Constructor for initializing point coordinates.")
      .def_property(
          "x", [](const mrpt::poses::CPoint3D &p) { return p.x(); },
          [](mrpt::poses::CPoint3D &p, double v) { p.x() = v; })
      .def_property(
          "y", [](const mrpt::poses::CPoint3D &p) { return p.y(); },
          [](mrpt::poses::CPoint3D &p, double v) { p.y() = v; })
      .def_property(
          "z", [](const mrpt::poses::CPoint3D &p) { return p.z(); },
          [](mrpt::poses::CPoint3D &p, double v) { p.z() = v; })
      .def(
          "asString", &mrpt::poses::CPoint3D::asString,
          "Returns a text representation, e.g. \"[0.02 1.04 -0.80]\".")
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
      std::shared_ptr<mrpt::poses::CPose3DQuat>>(
      m, "CPose3DQuat",
      "A class used to store a 3D pose as a translation (x,y,z) and a quaternion (qr,qx,qy,qz).")
      .def(
          py::init<>(),
          "Default constructor, initialize translation to zeros and quaternion to no rotation.")
      .def(py::init<const mrpt::poses::CPose3D &>(), "Builds the pose from a CPose3D.")
      .def(
          py::init<const mrpt::math::TPose3DQuat &>(), py::arg("p"),
          "Builds the pose from a lightweight mrpt.math.TPose3DQuat.")
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
      .def(
          "norm", &mrpt::poses::CPose3DQuat::norm,
          "Returns the Euclidean norm of the translation part.")
      .def(
          "asString", &mrpt::poses::CPose3DQuat::asString,
          "Returns a human-readable textual representation of the object as: \"[x y z qw qx qy "
          "qz]\".")
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
      .def(
          -py::self,
          "Returns the inverse pose as a new object (SE(3) group inverse, not the negation of "
          "each coordinate).")
      .def("__str__", &mrpt::poses::CPose3DQuat::asString)
      .def("__repr__", &mrpt::poses::CPose3DQuat::asString);

  // -------------------------------------------------------------------------
  // CPosePDFGaussian
  // -------------------------------------------------------------------------
  py::class_<
      mrpt::poses::CPosePDFGaussian, mrpt::poses::CPosePDF,
      std::shared_ptr<mrpt::poses::CPosePDFGaussian>>(
      m, "CPosePDFGaussian",
      "A PDF of a 2D pose as a Gaussian with a mean and a 3x3 covariance matrix.")
      .def(py::init<>(), "Default constructor.")
      .def(
          py::init<const mrpt::poses::CPose2D &>(),
          "Builds the PDF from a mean, with zero covariance.")
      .def(
          py::init<const mrpt::poses::CPose2D &, const mrpt::math::CMatrixDouble33 &>(),
          "Builds the PDF from a mean and a 3x3 covariance matrix.")
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
      .def(
          "evaluatePDF", &mrpt::poses::CPosePDFGaussian::evaluatePDF,
          "Evaluates the PDF at a given point.")
      .def(
          "evaluateNormalizedPDF", &mrpt::poses::CPosePDFGaussian::evaluateNormalizedPDF,
          "Evaluates the ratio PDF(x) / PDF(MEAN), that is, the normalized PDF in the range [0,1].")
      .def(
          py::self + py::self,
          "Pose composition a (+) b, propagating both covariances (independent variables).")
      .def(
          mrpt::poses::CPose2D() + py::self,
          "Composition p (+) pdf of a pose p without uncertainty and this PDF.")
      .def(
          py::self - py::self,
          "Inverse pose composition a (-) b, propagating both covariances (independent "
          "variables).")
      // Lambdas, since the C++ in-place operators return void:
      .def(
          "__iadd__",
          [](mrpt::poses::CPosePDFGaussian &self,
             const mrpt::poses::CPose2D &Ap) -> mrpt::poses::CPosePDFGaussian &
          {
            self += Ap;
            return self;
          },
          py::is_operator(), "Makes this = this (+) Ap, for a pose without uncertainty.")
      .def(
          "__iadd__",
          [](mrpt::poses::CPosePDFGaussian &self,
             const mrpt::poses::CPosePDFGaussian &Ap) -> mrpt::poses::CPosePDFGaussian &
          {
            self += Ap;
            return self;
          },
          py::is_operator(), "Makes this = this (+) Ap, propagating both covariances.")
      .def(
          "__isub__",
          [](mrpt::poses::CPosePDFGaussian &self,
             const mrpt::poses::CPosePDFGaussian &ref) -> mrpt::poses::CPosePDFGaussian &
          {
            self -= ref;
            return self;
          },
          py::is_operator(), "Makes this = this (-) ref, propagating both covariances.")
      .def(
          "inverse",
          [](const mrpt::poses::CPosePDFGaussian &self)
          {
            mrpt::poses::CPosePDFGaussian out;
            self.inverse(out);
            return out;
          },
          "Returns the PDF of the inverse pose, (0,0,0) (-) this, as a new object. This PDF is "
          "not modified.")
      .def(
          "inverseComposition",
          py::overload_cast<
              const mrpt::poses::CPosePDFGaussian &, const mrpt::poses::CPosePDFGaussian &>(
              &mrpt::poses::CPosePDFGaussian::inverseComposition),
          "x"_a, "ref"_a,
          "Sets this = x (-) ref, propagating both covariances (independent variables).")
      .def(
          "inverseComposition",
          py::overload_cast<
              const mrpt::poses::CPosePDFGaussian &, const mrpt::poses::CPosePDFGaussian &,
              const mrpt::math::CMatrixDouble33 &>(
              &mrpt::poses::CPosePDFGaussian::inverseComposition),
          "x1"_a, "x0"_a, "COV_01"_a,
          "Sets this = x1 (-) x0, given the 3x3 cross-covariance COV_01 = E[(x0-m0)(x1-m1)^T].")
      .def(
          "changeCoordinatesReference",
          py::overload_cast<const mrpt::poses::CPose2D &>(
              &mrpt::poses::CPosePDFGaussian::changeCoordinatesReference),
          "newReferenceBase"_a,
          "Makes this = newReferenceBase (+) this, for a reference pose without uncertainty.")
      .def(
          "changeCoordinatesReference",
          py::overload_cast<const mrpt::poses::CPose3D &>(
              &mrpt::poses::CPosePDFGaussian::changeCoordinatesReference),
          "newReferenceBase"_a,
          "Makes this = newReferenceBase (+) this, using the 2D part of a 3D reference pose.")
      .def("__str__", &mrpt::poses::CPosePDFGaussian::asString)
      .def("__repr__", &mrpt::poses::CPosePDFGaussian::asString);

  // -------------------------------------------------------------------------
  // CPosePDFGaussianInf
  // -------------------------------------------------------------------------
  py::class_<
      mrpt::poses::CPosePDFGaussianInf, mrpt::poses::CPosePDF,
      std::shared_ptr<mrpt::poses::CPosePDFGaussianInf>>(
      m, "CPosePDFGaussianInf",
      "A PDF of a 2D pose as a Gaussian with a mean and a 3x3 information (inverse covariance) "
      "matrix.")
      .def(py::init<>(), "Default constructor: zero mean and zero information matrix.")
      .def(
          py::init<const mrpt::poses::CPose2D &>(),
          "Builds the PDF from a mean, with zero information matrix.")
      .def(
          py::init<const mrpt::poses::CPose2D &, const mrpt::math::CMatrixDouble33 &>(),
          py::arg("mean"), py::arg("inf_matrix"),
          "Builds the PDF from a mean and a 3x3 information matrix.")
      .def_readwrite("mean", &mrpt::poses::CPosePDFGaussianInf::mean)
      .def_readwrite("cov_inv", &mrpt::poses::CPosePDFGaussianInf::cov_inv)
      .def(
          "drawSingleSample",
          [](const mrpt::poses::CPosePDFGaussianInf &self)
          {
            mrpt::poses::CPose2D out;
            self.drawSingleSample(out);
            return out;
          },
          "Draws a single sample from the distribution.")
      .def("__str__", &mrpt::poses::CPosePDFGaussianInf::asString)
      .def("__repr__", &mrpt::poses::CPosePDFGaussianInf::asString);

  // -------------------------------------------------------------------------
  // CPosePDFSOG
  // -------------------------------------------------------------------------
  using mrpt::poses::CPosePDFSOG;
  py::class_<CPosePDFSOG, mrpt::poses::CPosePDF, std::shared_ptr<CPosePDFSOG>> clSOG(
      m, "CPosePDFSOG",
      "A PDF of a 2D pose as a Sum of Gaussians (SOG): a set of weighted Gaussian modes.");

  py::class_<CPosePDFSOG::TGaussianMode>(clSOG, "TGaussianMode", "One mode of a CPosePDFSOG.")
      .def(py::init<>(), "Default constructor.")
      .def_readwrite("mean", &CPosePDFSOG::TGaussianMode::mean, "Mean of the mode (CPose2D).")
      .def_readwrite("cov", &CPosePDFSOG::TGaussianMode::cov, "3x3 covariance matrix.")
      .def_readwrite(
          "log_w", &CPosePDFSOG::TGaussianMode::log_w, "Logarithm of the weight of the mode.")
      .def(
          "__repr__",
          [](const CPosePDFSOG::TGaussianMode &g) {
            return "TGaussianMode(mean=" + g.mean.asString() +
                   ", log_w=" + std::to_string(g.log_w) + ")";
          });

  clSOG.def(py::init<size_t>(), "nModes"_a = 1, "Creates the given number of modes.")
      .def("size", &CPosePDFSOG::size, "Returns the number of modes.")
      .def("__len__", &CPosePDFSOG::size)
      .def("empty", &CPosePDFSOG::empty, "Returns true if there are no modes.")
      .def("clear", &CPosePDFSOG::clear, "Removes all the modes.")
      .def("resize", &CPosePDFSOG::resize, "N"_a, "Sets the number of modes.")
      .def("push_back", &CPosePDFSOG::push_back, "mode"_a, "Appends a mode (TGaussianMode).")
      .def(
          "get",
          [](CPosePDFSOG &self, size_t i) -> CPosePDFSOG::TGaussianMode & { return self.get(i); },
          "i"_a, py::return_value_policy::reference_internal,
          "Returns a reference to the i'th mode.")
      .def(
          "__getitem__",
          [](CPosePDFSOG &self, size_t i) -> CPosePDFSOG::TGaussianMode &
          {
            if (i >= self.size())
            {
              throw py::index_error("mode index out of range");
            }
            return self.get(i);
          },
          py::return_value_policy::reference_internal)
      .def(
          "__iter__", [](CPosePDFSOG &self) { return py::make_iterator(self.begin(), self.end()); },
          py::keep_alive<0, 1>())
      .def(
          "normalizeWeights", &CPosePDFSOG::normalizeWeights,
          "Normalizes the weights of the modes, so that they sum up to one.")
      .def(
          "mergeModes", &CPosePDFSOG::mergeModes, "max_KLd"_a = 0.5, "verbose"_a = false,
          "Merges very close modes, up to a maximum Kullback-Leibler distance.")
      .def(
          "getMostLikelyCovarianceAndMean",
          [](const CPosePDFSOG &self)
          {
            const auto [cov, mean] = self.getMostLikelyCovarianceAndMean();
            return py::make_tuple(cov, mean);
          },
          "Returns the tuple (cov: CMatrixDouble33, mean: CPose2D) of the mode with the highest "
          "weight.")
      .def(
          "evaluatePDF", &CPosePDFSOG::evaluatePDF, "x"_a, "sumOverAllPhis"_a = false,
          "Evaluates the PDF at a given pose.")
      .def(
          "evaluateNormalizedPDF", &CPosePDFSOG::evaluateNormalizedPDF, "x"_a,
          "Evaluates the ratio PDF(x) / max(PDF), in the range [0,1].")
      .def(
          "__repr__", [](const CPosePDFSOG &self)
          { return "CPosePDFSOG(" + std::to_string(self.size()) + " modes)"; });

  // -------------------------------------------------------------------------
  // TInterpolatorMethod
  // -------------------------------------------------------------------------
  py::enum_<mrpt::poses::TInterpolatorMethod>(
      m, "TInterpolatorMethod",
      "Interpolation method of CPose2DInterpolator and CPose3DInterpolator. In 3D, all methods "
      "but the Slerp ones interpolate yaw, pitch and roll independently.")
      .value(
          "imSpline", mrpt::poses::imSpline,
          "Spline interpolation using 4 points (2 before and 2 after the query time).")
      .value(
          "imLinear2Neig", mrpt::poses::imLinear2Neig,
          "Linear interpolation between the previous and next neighbors.")
      .value(
          "imLinear4Neig", mrpt::poses::imLinear4Neig,
          "Linear fit of the 4 closest points (2 before and 2 after the query time).")
      .value(
          "imSSLLLL", mrpt::poses::imSSLLLL,
          "Spline for x and y, linear least squares for z, yaw, pitch and roll.")
      .value(
          "imSSLSLL", mrpt::poses::imSSLSLL,
          "Spline for x, y and yaw, linear least squares for z, pitch and roll.")
      .value(
          "imLinearSlerp", mrpt::poses::imLinearSlerp,
          "Linear for x, y and z, Slerp for the rotation (the default).")
      .value(
          "imSplineSlerp", mrpt::poses::imSplineSlerp,
          "Spline for x, y and z (as imSpline), Slerp for the rotation.")
      .export_values();

  // -------------------------------------------------------------------------
  // CPose2DInterpolator
  // -------------------------------------------------------------------------
  py::class_<
      mrpt::poses::CPose2DInterpolator, mrpt::serialization::CSerializable,
      std::shared_ptr<mrpt::poses::CPose2DInterpolator>>(
      m, "CPose2DInterpolator",
      "A time-stamped trajectory in SE(2), with interpolation between poses.")
      .def(py::init<>(), "Default constructor.")
      .def(
          "insert",
          py::overload_cast<const mrpt::Clock::time_point &, const mrpt::math::TPose2D &>(
              &mrpt::poses::CPose2DInterpolator::insert),
          "Inserts a new pose in the sequence. It overwrites any previously existing pose at "
          "exactly the same time.")
      .def(
          "interpolate",
          [](const mrpt::poses::CPose2DInterpolator &self, const mrpt::Clock::time_point &t)
          {
            mrpt::math::TPose2D out;
            bool valid = false;
            self.interpolate(t, out, valid);
            return py::make_tuple(out, valid);
          },
          "Returns (TPose2D, valid): interpolated pose at given time")
      .def(
          "size", &mrpt::poses::CPose2DInterpolator::size,
          "Returns the number of poses in the trajectory.")
      .def(
          "empty", &mrpt::poses::CPose2DInterpolator::empty,
          "Returns true if the trajectory has no poses.")
      .def(
          "clear", &mrpt::poses::CPose2DInterpolator::clear,
          "Clears the current sequence of poses.")
      .def(
          "setInterpolationMethod", &mrpt::poses::CPose2DInterpolator::setInterpolationMethod,
          "method"_a, "Sets the interpolation method (default: imLinearSlerp).")
      .def(
          "getInterpolationMethod", &mrpt::poses::CPose2DInterpolator::getInterpolationMethod,
          "Returns the current interpolation method.")
      .def("__len__", [](const mrpt::poses::CPose2DInterpolator &self) { return self.size(); });

  // -------------------------------------------------------------------------
  // CPose3DInterpolator
  // -------------------------------------------------------------------------
  py::class_<
      mrpt::poses::CPose3DInterpolator, mrpt::serialization::CSerializable,
      std::shared_ptr<mrpt::poses::CPose3DInterpolator>>(
      m, "CPose3DInterpolator",
      "A time-stamped trajectory in SE(3), with interpolation between poses.")
      .def(py::init<>(), "Default constructor.")
      .def(
          "insert",
          py::overload_cast<const mrpt::Clock::time_point &, const mrpt::math::TPose3D &>(
              &mrpt::poses::CPose3DInterpolator::insert),
          "Inserts a new pose in the sequence. It overwrites any previously existing pose at "
          "exactly the same time.")
      .def(
          "interpolate",
          [](const mrpt::poses::CPose3DInterpolator &self, const mrpt::Clock::time_point &t)
          {
            mrpt::math::TPose3D out;
            bool valid = false;
            self.interpolate(t, out, valid);
            return py::make_tuple(out, valid);
          },
          "Returns (TPose3D, valid): interpolated pose at given time")
      .def(
          "size", &mrpt::poses::CPose3DInterpolator::size,
          "Returns the number of poses in the trajectory.")
      .def(
          "empty", &mrpt::poses::CPose3DInterpolator::empty,
          "Returns true if the trajectory has no poses.")
      .def(
          "clear", &mrpt::poses::CPose3DInterpolator::clear,
          "Clears the current sequence of poses.")
      .def(
          "setInterpolationMethod", &mrpt::poses::CPose3DInterpolator::setInterpolationMethod,
          "method"_a, "Sets the interpolation method (default: imLinearSlerp).")
      .def(
          "getInterpolationMethod", &mrpt::poses::CPose3DInterpolator::getInterpolationMethod,
          "Returns the current interpolation method.")
      .def("__len__", [](const mrpt::poses::CPose3DInterpolator &self) { return self.size(); });

  // -------------------------------------------------------------------------
  // CPoseRandomSampler
  // -------------------------------------------------------------------------
  py::class_<mrpt::poses::CPoseRandomSampler, std::shared_ptr<mrpt::poses::CPoseRandomSampler>>(
      m, "CPoseRandomSampler",
      "An efficient generator of random samples drawn from a given 2D (CPosePDF) or 3D "
      "(CPose3DPDF) pose probability density function (pdf).")
      .def(py::init<>(), "Default constructor.")
      .def(
          "setPosePDF",
          [](mrpt::poses::CPoseRandomSampler &self,
             const std::shared_ptr<mrpt::poses::CPosePDF> &pdf) { self.setPosePDF(*pdf); },
          "Sets the 2D pose PDF to draw samples from.")
      .def(
          "setPosePDF",
          [](mrpt::poses::CPoseRandomSampler &self,
             const std::shared_ptr<mrpt::poses::CPose3DPDF> &pdf) { self.setPosePDF(*pdf); },
          "Sets the 3D pose PDF to draw samples from.")
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

  // -------------------------------------------------------------------------
  // Copies: copy constructor, __copy__ and __deepcopy__
  // -------------------------------------------------------------------------
  addCopyMethods<mrpt::poses::CPose2D>();
  addCopyMethods<mrpt::poses::CPose3D>();
  addCopyMethods<mrpt::poses::CPose3DQuat>();
  addCopyMethods<mrpt::poses::CPoint2D>();
  addCopyMethods<mrpt::poses::CPoint3D>();
  addCopyMethods<mrpt::poses::CPosePDFGaussian>();
  addCopyMethods<mrpt::poses::CPosePDFGaussianInf>();
  addCopyMethods<mrpt::poses::CPose3DPDFGaussian>();
  addCopyMethods<mrpt::poses::CPose3DPDFGaussianInf>();
  addCopyMethods<mrpt::poses::CPosePDFParticles>();
  addCopyMethods<mrpt::poses::CPose3DPDFParticles>();
  addCopyMethods<mrpt::poses::CPosePDFSOG>();
  addCopyMethods<mrpt::poses::CPose2DInterpolator>();
  addCopyMethods<mrpt::poses::CPose3DInterpolator>();
}