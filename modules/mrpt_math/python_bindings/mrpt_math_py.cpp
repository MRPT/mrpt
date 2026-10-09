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

/* -------------------------------------------------------------------------
 * Mobile Robot Programming Toolkit (MRPT)
 * https://github.com/MRPT/mrpt/
 * ------------------------------------------------------------------------- */

#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

// MRPT math headers
#include <mrpt/core/format.h>
#include <mrpt/math/CHistogram.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CPolygon.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/CVectorFixed.h>
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/math/TLine2D.h>
#include <mrpt/math/TLine3D.h>
#include <mrpt/math/TObject2D.h>
#include <mrpt/math/TObject3D.h>
#include <mrpt/math/TPlane.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TPose3DQuat.h>
#include <mrpt/math/TSegment2D.h>
#include <mrpt/math/TSegment3D.h>
#include <mrpt/math/TTwist2D.h>
#include <mrpt/math/TTwist3D.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/serialization/CSerializable.h>

namespace py = pybind11;

// Define RowMajor Eigen types to match MRPT's internal layout
// This facilitates zero-copy mapping to NumPy
using EigenRowMatrixXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using EigenRowVectorXd = Eigen::Matrix<double, 1, Eigen::Dynamic, Eigen::RowMajor>;

namespace
{
/** * Helper to register any CMatrixFixed (including CVectorFixed) with NumPy support.
 * It detects dimensions and uses Eigen::Map with RowMajor to ensure zero-copy.
 */
template <typename T>
void bind_mrpt_fixed_type(py::module& m, const std::string& name)
{
  const std::string doc = std::to_string(T::RowsAtCompileTime) + "x" +
                          std::to_string(T::ColsAtCompileTime) +
                          " fixed-size matrix of doubles, convertible to and from NumPy arrays.";
  py::class_<T, std::shared_ptr<T>>(m, name.c_str(), doc.c_str())
      .def(py::init<>(), "Builds a matrix filled with zeros.")
      .def(
          py::init(
              [](const typename T::eigen_t& src)
              {
                auto mat = std::make_shared<T>();
                mat->asEigen() = src;
                return mat;
              }),
          py::arg("array"), "Builds the matrix from a NumPy array of the same shape")
      .def(
          "as_numpy", [](const T& self) -> typename T::eigen_t { return self.asEigen(); },
          "Returns the matrix as a NumPy array.")
      .def(
          "__array__",
          [](const T& self, py::object /*dtype*/, py::object /*copy*/) -> typename T::eigen_t
          { return self.asEigen(); },
          py::arg("dtype") = py::none(), py::arg("copy") = py::none())
      .def("__repr__", [name](const T& self) { return "[" + name + "]\n" + self.asString(); });
}
}  // namespace

PYBIND11_MODULE(_bindings, m)
{
  m.doc() = "Python bindings for mrpt_math (with NumPy support)";

  // -------------------------------------------------------------------------
  // 1. CMatrixDouble (Dynamic size)
  // -------------------------------------------------------------------------
  py::class_<mrpt::math::CMatrixDouble, std::shared_ptr<mrpt::math::CMatrixDouble>>(
      m, "CMatrixDouble", "A dynamic-size matrix of doubles, convertible to and from NumPy arrays.")
      .def(py::init<>(), "Default constructor.")
      .def(py::init<int, int>(), "Builds a matrix of the given size (rows, cols).")
      // Automatic conversion: NumPy -> CMatrixDouble
      .def(
          py::init(
              [](const EigenRowMatrixXd& src)
              {
                auto mat = std::make_shared<mrpt::math::CMatrixDouble>(src.rows(), src.cols());
                mat->asEigen() = src.cast<double>();
                return mat;
              }),
          "Builds the matrix from a 2D NumPy array.")
      // Manual conversion: mrpt_obj.as_numpy()
      .def(
          "as_numpy",
          [](const mrpt::math::CMatrixDouble& self)
          {
            // Return an Eigen Map that NumPy can consume directly
            return Eigen::Map<const EigenRowMatrixXd>(&self(0, 0), self.rows(), self.cols());
          },
          "Returns the matrix as a NumPy array.")
      .def("__repr__", [](const mrpt::math::CMatrixDouble& self) { return self.asString(); });

  // -------------------------------------------------------------------------
  // 2. CVectorDouble (Dynamic size)
  // -------------------------------------------------------------------------
  py::class_<mrpt::math::CVectorDouble, std::shared_ptr<mrpt::math::CVectorDouble>>(
      m, "CVectorDouble", "A dynamic-size vector of doubles, convertible to and from NumPy arrays.")
      .def(py::init<>(), "Default constructor.")
      .def(py::init<size_t>(), "Builds a vector of the given length.")
      .def(
          py::init(
              [](const EigenRowVectorXd& src)
              {
                auto v = std::make_shared<mrpt::math::CVectorDouble>();
                v->resize(src.size());
                for (int i = 0; i < src.size(); ++i)
                {
                  (*v)[i] = src[i];
                }
                return v;
              }),
          "Builds the vector from a 1D NumPy array.")
      .def(
          "as_numpy", [](const mrpt::math::CVectorDouble& self) { return self.asEigen(); },
          "Returns the vector as a NumPy array.");

  // --- 1. Fixed-Size Matrices (from CMatrixFixed.h) ---
  bind_mrpt_fixed_type<mrpt::math::CMatrixDouble22>(m, "CMatrixDouble22");
  bind_mrpt_fixed_type<mrpt::math::CMatrixDouble33>(m, "CMatrixDouble33");
  bind_mrpt_fixed_type<mrpt::math::CMatrixDouble44>(m, "CMatrixDouble44");
  bind_mrpt_fixed_type<mrpt::math::CMatrixDouble66>(m, "CMatrixDouble66");
  bind_mrpt_fixed_type<mrpt::math::CMatrixDouble77>(m, "CMatrixDouble77");

  // --- 2. Fixed-Size Vectors (from CVectorFixed.h) ---
  // Note: CVectorFixedDouble<N> is just an alias for CMatrixFixed<double, N, 1>
  bind_mrpt_fixed_type<mrpt::math::CVectorFixedDouble<2>>(m, "CVectorFixedDouble2");
  bind_mrpt_fixed_type<mrpt::math::CVectorFixedDouble<3>>(m, "CVectorFixedDouble3");
  bind_mrpt_fixed_type<mrpt::math::CVectorFixedDouble<6>>(m, "CVectorFixedDouble6");

  // -------------------------------------------------------------------------
  // CQuaternionDouble
  // -------------------------------------------------------------------------
  using mrpt::math::CQuaternionDouble;
  py::class_<CQuaternionDouble, std::shared_ptr<CQuaternionDouble>>(
      m, "CQuaternionDouble",
      "A unit quaternion (r, x, y, z) for 3D rotations, with r the real part.")
      .def(py::init<>(), "Identity rotation (1, 0, 0, 0).")
      .def(
          py::init<double, double, double, double>(), py::arg("r"), py::arg("x"), py::arg("y"),
          py::arg("z"), "Builds the quaternion from its components (it must be normalized).")
      .def_property(
          "r", [](const CQuaternionDouble& q) { return q.r(); },
          [](CQuaternionDouble& q, double v) { q.r(v); }, "Real part")
      .def_property(
          "x", [](const CQuaternionDouble& q) { return q.x(); },
          [](CQuaternionDouble& q, double v) { q.x(v); }, "Imaginary part, i")
      .def_property(
          "y", [](const CQuaternionDouble& q) { return q.y(); },
          [](CQuaternionDouble& q, double v) { q.y(v); }, "Imaginary part, j")
      .def_property(
          "z", [](const CQuaternionDouble& q) { return q.z(); },
          [](CQuaternionDouble& q, double v) { q.z(v); }, "Imaginary part, k")
      .def("normalize", &CQuaternionDouble::normalize, "Normalizes the quaternion to unit norm.")
      .def("normSqr", &CQuaternionDouble::normSqr, "Squared norm of the quaternion.")
      .def(
          "ensurePositiveRealPart", &CQuaternionDouble::ensurePositiveRealPart,
          "Flips the sign of all components if needed, so that r >= 0 (same rotation).")
      .def(
          "conj", py::overload_cast<>(&CQuaternionDouble::conj, py::const_),
          "Returns the conjugate quaternion (the inverse rotation).")
      .def(
          "rpy",
          [](const CQuaternionDouble& q)
          {
            double roll = 0;
            double pitch = 0;
            double yaw = 0;
            q.rpy(roll, pitch, yaw);
            return py::make_tuple(roll, pitch, yaw);
          },
          "Returns the equivalent (roll, pitch, yaw) angles, in radians.")
      .def(
          "rotationMatrix",
          [](const CQuaternionDouble& q)
          { return q.rotationMatrix<mrpt::math::CMatrixDouble33>(); },
          "Returns the equivalent 3x3 rotation matrix.")
      .def(
          "rotatePoint",
          [](const CQuaternionDouble& q, double lx, double ly, double lz)
          {
            double gx = 0;
            double gy = 0;
            double gz = 0;
            q.rotatePoint(lx, ly, lz, gx, gy, gz);
            return py::make_tuple(gx, gy, gz);
          },
          py::arg("x"), py::arg("y"), py::arg("z"), "Rotates a 3D point; returns (x, y, z).")
      .def(
          "as_numpy", [](const CQuaternionDouble& q) -> Eigen::Vector4d { return q.asEigen(); },
          "Returns the components [r, x, y, z] as a NumPy array.")
      .def(
          "__mul__",
          [](const CQuaternionDouble& a, const CQuaternionDouble& b)
          {
            CQuaternionDouble ret;
            ret.crossProduct(a, b);
            return ret;
          },
          py::is_operator())
      .def(
          "__repr__",
          [](const CQuaternionDouble& q) {
            return mrpt::format(
                "CQuaternionDouble(r=%g, x=%g, y=%g, z=%g)", q.r(), q.x(), q.y(), q.z());
          });

  // -------------------------------------------------------------------------
  // Lightweight pose types
  // -------------------------------------------------------------------------
  // -------------------------------------------------------------------------
  // TPoint2D
  // -------------------------------------------------------------------------
  // The float variants are registered first, since cast_float() returns them:
  py::class_<mrpt::math::TPoint2Df> point2Df(
      m, "TPoint2Df", "A 2D point (x, y), with float coordinates.");
  py::class_<mrpt::math::TPoint3Df> point3Df(
      m, "TPoint3Df", "A 3D point (x, y, z), with float coordinates.");

  py::class_<mrpt::math::TPoint2D>(m, "TPoint2D", "A 2D point (x, y), with double coordinates.")
      .def(py::init<>(), "Default constructor. Initializes to zeros.")
      .def(py::init<double, double>(), "Constructor from coordinates.")
      .def(
          py::init(
              [](const std::vector<double>& v)
              {
                if (v.size() != 2)
                {
                  throw std::invalid_argument("List must have 2 elements [x, y]");
                }
                return mrpt::math::TPoint2D(v[0], v[1]);
              }),
          "Builds the point from a list [x, y].")
      .def_readwrite("x", &mrpt::math::TPoint2D::x)
      .def_readwrite("y", &mrpt::math::TPoint2D::y)
      .def("norm", &mrpt::math::TPoint2D::norm, "Returns the norm sqrt(x^2 + y^2).")
      .def("sqrNorm", &mrpt::math::TPoint2D::sqrNorm, "Returns the squared norm x^2 + y^2.")
      .def(
          "unitarize", &mrpt::math::TPoint2D::unitarize,
          "Returns this vector with unit length: v/norm(v)")
      .def(
          "cast_float", [](const mrpt::math::TPoint2D& p) { return p.cast<float>(); },
          "Returns a copy with float coordinates (TPoint2Df).")
      .def(py::self + py::self)
      .def(py::self - py::self)
      .def(py::self += py::self)
      .def(py::self -= py::self)
      .def(py::self * double())
      .def(py::self / double())
      .def(-py::self)
      .def(py::self == py::self)
      .def(py::self != py::self)
      .def(
          "__array__",
          [](const mrpt::math::TPoint2D& p, py::object /*dtype*/, py::object /*copy*/)
          { return py::array_t<double>({2}, {sizeof(double)}, &p.x); },
          py::arg("dtype") = py::none(), py::arg("copy") = py::none())
      .def("__len__", [](const mrpt::math::TPoint2D&) { return 2; })
      .def("__repr__", [](const mrpt::math::TPoint2D& p) { return p.asString(); })
      .def("__str__", [](const mrpt::math::TPoint2D& p) { return p.asString(); });

  // -------------------------------------------------------------------------
  // TPoint3D
  // -------------------------------------------------------------------------
  py::class_<mrpt::math::TPoint3D>(m, "TPoint3D", "A 3D point (x, y, z), with double coordinates.")
      .def(py::init<>(), "Default constructor. Initializes to zeros.")
      .def(py::init<double, double, double>(), "Constructor from coordinates.")
      .def(
          py::init(
              [](const std::vector<double>& v)
              {
                if (v.size() != 3)
                {
                  throw std::invalid_argument("List must have 3 elements [x, y, z]");
                }
                return mrpt::math::TPoint3D(v[0], v[1], v[2]);
              }),
          "Builds the point from a list [x, y, z].")
      .def_readwrite("x", &mrpt::math::TPoint3D::x)
      .def_readwrite("y", &mrpt::math::TPoint3D::y)
      .def_readwrite("z", &mrpt::math::TPoint3D::z)
      .def("norm", &mrpt::math::TPoint3D::norm, "Returns the norm sqrt(x^2 + y^2 + z^2).")
      .def("sqrNorm", &mrpt::math::TPoint3D::sqrNorm, "Returns the squared norm x^2 + y^2 + z^2.")
      .def(
          "unitarize", &mrpt::math::TPoint3D::unitarize,
          "Returns this vector with unit length: v/norm(v)")
      .def("dot", &mrpt::math::TPoint3D::dot, "Scalar product s=dot(this,p)")
      .def("cross", &mrpt::math::TPoint3D::cross, "Cross product res = cross(this, p)")
      .def(
          "cast_float", [](const mrpt::math::TPoint3D& p) { return p.cast<float>(); },
          "Returns a copy with float coordinates (TPoint3Df).")
      .def(py::self + py::self)
      .def(py::self - py::self)
      .def(py::self += py::self)
      .def(py::self -= py::self)
      .def(py::self * double())
      .def(py::self / double())
      .def(-py::self)
      .def(py::self == py::self)
      .def(py::self != py::self)
      .def(
          "__array__",
          [](const mrpt::math::TPoint3D& p, py::object /*dtype*/, py::object /*copy*/)
          { return py::array_t<double>({3}, {sizeof(double)}, &p.x); },
          py::arg("dtype") = py::none(), py::arg("copy") = py::none())
      .def("__len__", [](const mrpt::math::TPoint3D&) { return 3; })
      .def("__repr__", [](const mrpt::math::TPoint3D& p) { return p.asString(); })
      .def("__str__", [](const mrpt::math::TPoint3D& p) { return p.asString(); });

  // -------------------------------------------------------------------------
  // TPoint2Df
  // -------------------------------------------------------------------------
  point2Df.def(py::init<>(), "Default constructor. Initializes to zeros.")
      .def(py::init<float, float>(), "Constructor from coordinates.")
      .def(
          py::init(
              [](const std::vector<float>& v)
              {
                if (v.size() != 2)
                {
                  throw std::invalid_argument("List must have 2 elements [x, y]");
                }
                return mrpt::math::TPoint2Df(v[0], v[1]);
              }),
          "Builds the point from a list [x, y].")
      .def_readwrite("x", &mrpt::math::TPoint2Df::x)
      .def_readwrite("y", &mrpt::math::TPoint2Df::y)
      .def("norm", &mrpt::math::TPoint2Df::norm, "Returns the norm sqrt(x^2 + y^2).")
      .def("sqrNorm", &mrpt::math::TPoint2Df::sqrNorm, "Returns the squared norm x^2 + y^2.")
      .def(
          "unitarize", &mrpt::math::TPoint2Df::unitarize,
          "Returns this vector with unit length: v/norm(v)")
      .def(
          "cast_double", [](const mrpt::math::TPoint2Df& p) { return p.cast<double>(); },
          "Returns a copy with double coordinates (TPoint2D).")
      .def(py::self + py::self)
      .def(py::self - py::self)
      .def(py::self == py::self)
      .def(py::self != py::self)
      .def(
          "__array__",
          [](const mrpt::math::TPoint2Df& p, py::object /*dtype*/, py::object /*copy*/)
          { return py::array_t<float>({2}, {sizeof(float)}, &p.x); },
          py::arg("dtype") = py::none(), py::arg("copy") = py::none())
      .def("__len__", [](const mrpt::math::TPoint2Df&) { return 2; })
      .def("__repr__", [](const mrpt::math::TPoint2Df& p) { return p.asString(); })
      .def("__str__", [](const mrpt::math::TPoint2Df& p) { return p.asString(); });

  // -------------------------------------------------------------------------
  // TPoint3Df
  // -------------------------------------------------------------------------
  point3Df.def(py::init<>(), "Default constructor. Initializes to zeros.")
      .def(py::init<float, float, float>(), "Constructor from coordinates.")
      .def(
          py::init(
              [](const std::vector<float>& v)
              {
                if (v.size() != 3)
                {
                  throw std::invalid_argument("List must have 3 elements [x, y, z]");
                }
                return mrpt::math::TPoint3Df(v[0], v[1], v[2]);
              }),
          "Builds the point from a list [x, y, z].")
      .def_readwrite("x", &mrpt::math::TPoint3Df::x)
      .def_readwrite("y", &mrpt::math::TPoint3Df::y)
      .def_readwrite("z", &mrpt::math::TPoint3Df::z)
      .def("norm", &mrpt::math::TPoint3Df::norm, "Returns the norm sqrt(x^2 + y^2 + z^2).")
      .def("sqrNorm", &mrpt::math::TPoint3Df::sqrNorm, "Returns the squared norm x^2 + y^2 + z^2.")
      .def(
          "unitarize", &mrpt::math::TPoint3Df::unitarize,
          "Returns this vector with unit length: v/norm(v)")
      .def("dot", &mrpt::math::TPoint3Df::dot, "Scalar product s=dot(this,p)")
      .def("cross", &mrpt::math::TPoint3Df::cross, "Cross product res = cross(this, p)")
      .def(
          "cast_double", [](const mrpt::math::TPoint3Df& p) { return p.cast<double>(); },
          "Returns a copy with double coordinates (TPoint3D).")
      .def(py::self + py::self)
      .def(py::self - py::self)
      .def(py::self == py::self)
      .def(py::self != py::self)
      .def(
          "__array__",
          [](const mrpt::math::TPoint3Df& p, py::object /*dtype*/, py::object /*copy*/)
          { return py::array_t<float>({3}, {sizeof(float)}, &p.x); },
          py::arg("dtype") = py::none(), py::arg("copy") = py::none())
      .def("__len__", [](const mrpt::math::TPoint3Df&) { return 3; })
      .def("__repr__", [](const mrpt::math::TPoint3Df& p) { return p.asString(); })
      .def("__str__", [](const mrpt::math::TPoint3Df& p) { return p.asString(); });

  // -------------------------------------------------------------------------
  // TPose2D
  // -------------------------------------------------------------------------
  py::class_<mrpt::math::TPose2D>(
      m, "TPose2D", "Lightweight 2D pose (x, y, phi): an element of SE(2).")
      .def(py::init<>(), "Default fast constructor. Initializes to zeros.")
      .def(py::init<double, double, double>(), "Constructor from coordinates.")
      .def(
          py::init(
              [](const std::vector<double>& v)
              {
                if (v.size() != 3)
                {
                  throw std::invalid_argument("List must have 3 elements [x, y, phi]");
                }
                return mrpt::math::TPose2D(v[0], v[1], v[2]);
              }),
          "Builds the pose from a list [x, y, phi].")
      .def_readwrite("x", &mrpt::math::TPose2D::x)
      .def_readwrite("y", &mrpt::math::TPose2D::y)
      .def_readwrite("phi", &mrpt::math::TPose2D::phi)
      .def(
          "norm", &mrpt::math::TPose2D::norm,
          "Euclidean norm of the translation (x, y); phi is ignored.")
      .def(
          "normalizePhi", &mrpt::math::TPose2D::normalizePhi,
          "Wraps phi to the canonical range (-pi, pi].")
      .def(
          "composePoint", &mrpt::math::TPose2D::composePoint,
          "Transforms a point from the local frame of this pose into the global (world) frame.")
      .def(
          "inverseComposePoint", &mrpt::math::TPose2D::inverseComposePoint,
          "Transforms a point from the global (world) frame into the local frame of this pose.")
      .def(py::self + py::self)
      .def(py::self - py::self)
      .def(py::self == py::self)
      .def(py::self != py::self)
      .def(
          "__array__",
          [](const mrpt::math::TPose2D& p, py::object /*dtype*/, py::object /*copy*/)
          { return py::array_t<double>({3}, {sizeof(double)}, &p.x); },
          py::arg("dtype") = py::none(), py::arg("copy") = py::none())
      .def("__len__", [](const mrpt::math::TPose2D&) { return 3; })
      .def("__repr__", [](const mrpt::math::TPose2D& p) { return p.asString(); })
      .def("__str__", [](const mrpt::math::TPose2D& p) { return p.asString(); });

  // -------------------------------------------------------------------------
  // TPose3D
  // -------------------------------------------------------------------------
  py::class_<mrpt::math::TPose3D>(
      m, "TPose3D", "Lightweight 3D pose (x, y, z, yaw, pitch, roll): an element of SE(3).")
      .def(py::init<>(), "Default fast constructor. Initializes to zeros.")
      .def(
          py::init<double, double, double, double, double, double>(),
          "Constructor from coordinates.")
      .def(
          py::init(
              [](const std::vector<double>& v)
              {
                if (v.size() != 6)
                {
                  throw std::invalid_argument(
                      "List must have 6 elements [x, y, z, yaw, pitch, roll]");
                }
                return mrpt::math::TPose3D(v[0], v[1], v[2], v[3], v[4], v[5]);
              }),
          "Builds the pose from a list [x, y, z, yaw, pitch, roll].")
      .def_readwrite("x", &mrpt::math::TPose3D::x)
      .def_readwrite("y", &mrpt::math::TPose3D::y)
      .def_readwrite("z", &mrpt::math::TPose3D::z)
      .def_readwrite("yaw", &mrpt::math::TPose3D::yaw)
      .def_readwrite("pitch", &mrpt::math::TPose3D::pitch)
      .def_readwrite("roll", &mrpt::math::TPose3D::roll)
      .def(
          "norm", &mrpt::math::TPose3D::norm,
          "Euclidean norm of the translation (x, y, z); angles are ignored.")
      .def(
          "composePoint",
          py::overload_cast<const mrpt::math::TPoint3D&>(
              &mrpt::math::TPose3D::composePoint, py::const_),
          "Transforms a point from the local frame of this pose into the global (world) frame.")
      .def(
          "inverseComposePoint",
          py::overload_cast<const mrpt::math::TPoint3D&>(
              &mrpt::math::TPose3D::inverseComposePoint, py::const_),
          "Transforms a point from the global (world) frame into the local frame of this pose.")
      .def(
          "getRotationMatrix", [](const mrpt::math::TPose3D& p) { return p.getRotationMatrix(); },
          "Returns the 3x3 rotation matrix.")
      .def(
          "getHomogeneousMatrix",
          [](const mrpt::math::TPose3D& p) { return p.getHomogeneousMatrix(); },
          "Returns the 4x4 homogeneous transformation matrix.")
      .def(py::self + py::self)
      .def(py::self == py::self)
      .def(py::self != py::self)
      .def("__neg__", [](const mrpt::math::TPose3D& p) { return -p; })
      .def(
          "__array__",
          [](const mrpt::math::TPose3D& p, py::object /*dtype*/, py::object /*copy*/)
          { return py::array_t<double>({6}, {sizeof(double)}, &p.x); },
          py::arg("dtype") = py::none(), py::arg("copy") = py::none())
      .def("__len__", [](const mrpt::math::TPose3D&) { return 6; })
      .def("__repr__", [](const mrpt::math::TPose3D& p) { return p.asString(); })
      .def("__str__", [](const mrpt::math::TPose3D& p) { return p.asString(); });

  // =========================================================================
  // Geometry primitives (Phase 0.1 extensions)
  // =========================================================================

  // -------------------------------------------------------------------------
  // TSegment2D
  // -------------------------------------------------------------------------
  py::class_<mrpt::math::TSegment2D>(m, "TSegment2D", "2D segment, consisting of two points.")
      .def(py::init<>(), "Fast default constructor. Initializes to (0,0)-(0,0)")
      .def(
          py::init<const mrpt::math::TPoint2D&, const mrpt::math::TPoint2D&>(),
          "Constructor from both points.")
      .def_static(
          "FromPoints",
          [](const mrpt::math::TPoint2D& p1, const mrpt::math::TPoint2D& p2)
          { return mrpt::math::TSegment2D::FromPoints(p1, p2); },
          "Static method, returns segment from two points.")
      .def_readwrite("point1", &mrpt::math::TSegment2D::point1)
      .def_readwrite("point2", &mrpt::math::TSegment2D::point2)
      .def("length", &mrpt::math::TSegment2D::length, "Segment length.")
      .def(
          "distance",
          py::overload_cast<const mrpt::math::TPoint2D&>(
              &mrpt::math::TSegment2D::distance, py::const_),
          "Absolute distance to point.")
      .def(
          "contains", &mrpt::math::TSegment2D::contains,
          "Check whether a point is inside a segment.")
      .def("__repr__", [](const mrpt::math::TSegment2D& s) { return s.asString(); });

  // -------------------------------------------------------------------------
  // TSegment3D
  // -------------------------------------------------------------------------
  py::class_<mrpt::math::TSegment3D>(m, "TSegment3D", "3D segment, consisting of two points.")
      .def(py::init<>(), "Fast default constructor. Initializes to (0,0,0)-(0,0,0)")
      .def(
          py::init<const mrpt::math::TPoint3D&, const mrpt::math::TPoint3D&>(),
          "Constructor from two points.")
      .def_readwrite("point1", &mrpt::math::TSegment3D::point1)
      .def_readwrite("point2", &mrpt::math::TSegment3D::point2)
      .def("length", &mrpt::math::TSegment3D::length, "Segment length.")
      .def(
          "distance",
          py::overload_cast<const mrpt::math::TPoint3D&>(
              &mrpt::math::TSegment3D::distance, py::const_),
          "Distance to point.")
      .def(
          "contains", &mrpt::math::TSegment3D::contains,
          "Check whether a point is inside the segment.")
      .def("__repr__", [](const mrpt::math::TSegment3D& s) { return s.asString(); });

  // -------------------------------------------------------------------------
  // TLine2D
  // -------------------------------------------------------------------------
  py::class_<mrpt::math::TLine2D>(
      m, "TLine2D", "2D line without bounds, represented by its equation Ax+By+C=0.")
      .def(py::init<>(), "Fast default constructor. Initializes to undefined values.")
      .def(
          py::init<const mrpt::math::TPoint2D&, const mrpt::math::TPoint2D&>(),
          "Constructor from two points, through which the line will pass.")
      .def(
          py::init<double, double, double>(), py::arg("A"), py::arg("B"), py::arg("C"),
          "Constructor from line's coefficients.")
      .def_static(
          "FromTwoPoints",
          [](const mrpt::math::TPoint2D& p1, const mrpt::math::TPoint2D& p2)
          { return mrpt::math::TLine2D::FromTwoPoints(p1, p2); },
          "Static constructor from two points.")
      .def_property(
          "coefs",
          [](const mrpt::math::TLine2D& l)
          { return std::vector<double>(l.coefs.begin(), l.coefs.end()); },
          [](mrpt::math::TLine2D& l, const std::vector<double>& v)
          {
            if (v.size() != 3)
            {
              throw std::invalid_argument("coefs must have 3 elements");
            }
            std::copy(v.begin(), v.end(), l.coefs.begin());
          })
      .def(
          "evaluatePoint", &mrpt::math::TLine2D::evaluatePoint,
          "Evaluate point in the line's equation.")
      .def("contains", &mrpt::math::TLine2D::contains, "Check whether a point is inside the line.")
      .def("distance", &mrpt::math::TLine2D::distance, "Absolute distance from a given point.")
      .def("unitarize", &mrpt::math::TLine2D::unitarize, "Unitarize line's normal vector.")
      .def("__repr__", [](const mrpt::math::TLine2D& l) { return l.asString(); })
      .def("__str__", [](const mrpt::math::TLine2D& l) { return l.asString(); });

  // -------------------------------------------------------------------------
  // TLine3D
  // -------------------------------------------------------------------------
  py::class_<mrpt::math::TLine3D>(
      m, "TLine3D", "3D line, represented by a base point and a director vector.")
      .def(py::init<>(), "Fast default constructor. Initializes to all zeros.")
      .def(
          py::init<const mrpt::math::TPoint3D&, const mrpt::math::TPoint3D&>(),
          "Constructor from two points, through which the line will pass.")
      .def_static(
          "FromTwoPoints",
          [](const mrpt::math::TPoint3D& p1, const mrpt::math::TPoint3D& p2)
          { return mrpt::math::TLine3D::FromTwoPoints(p1, p2); },
          "Static constructor from two points.")
      .def_readwrite("pBase", &mrpt::math::TLine3D::pBase)
      .def_readwrite("director", &mrpt::math::TLine3D::director)
      .def("contains", &mrpt::math::TLine3D::contains, "Check whether a point is inside the line.")
      .def(
          "distance",
          py::overload_cast<const mrpt::math::TPoint3D&>(
              &mrpt::math::TLine3D::distance, py::const_),
          "Absolute distance between the line and a point.")
      .def("unitarize", &mrpt::math::TLine3D::unitarize, "Unitarize director vector.")
      .def(
          "closestPointTo", &mrpt::math::TLine3D::closestPointTo,
          "Closest point to p along the line. It is computed as the intersection of this with the "
          "plane perpendicular to this that passes through p.")
      .def("__repr__", [](const mrpt::math::TLine3D& l) { return l.asString(); })
      .def("__str__", [](const mrpt::math::TLine3D& l) { return l.asString(); });

  // -------------------------------------------------------------------------
  // TPlane
  // -------------------------------------------------------------------------
  py::class_<mrpt::math::TPlane>(m, "TPlane", "3D Plane, represented by its equation Ax+By+Cz+D=0.")
      .def(py::init<>(), "Fast default constructor (uninitialized coefficients).")
      .def(
          py::init<double, double, double, double>(), py::arg("A"), py::arg("B"), py::arg("C"),
          py::arg("D"), "Constructor from plane coefficients.")
      .def(
          py::init<
              const mrpt::math::TPoint3D&, const mrpt::math::TPoint3D&,
              const mrpt::math::TPoint3D&>(),
          "Defines a plane which contains these three points.")
      .def_static(
          "From3Points",
          [](const mrpt::math::TPoint3D& p1, const mrpt::math::TPoint3D& p2,
             const mrpt::math::TPoint3D& p3)
          { return mrpt::math::TPlane::From3Points(p1, p2, p3); },
          "Returns the plane that contains three points.")
      .def_property(
          "coefs",
          [](const mrpt::math::TPlane& pl)
          { return std::vector<double>(pl.coefs.begin(), pl.coefs.end()); },
          [](mrpt::math::TPlane& pl, const std::vector<double>& v)
          {
            if (v.size() != 4)
            {
              throw std::invalid_argument("coefs must have 4 elements");
            }
            std::copy(v.begin(), v.end(), pl.coefs.begin());
          })
      .def(
          "evaluatePoint", &mrpt::math::TPlane::evaluatePoint,
          "Evaluate a point in the plane's equation.")
      .def(
          "contains",
          py::overload_cast<const mrpt::math::TPoint3D&>(&mrpt::math::TPlane::contains, py::const_),
          "Check whether a point is contained into the plane.")
      .def(
          "distance",
          py::overload_cast<const mrpt::math::TPoint3D&>(&mrpt::math::TPlane::distance, py::const_),
          "Absolute distance to 3D point.")
      .def("unitarize", &mrpt::math::TPlane::unitarize, "Unitarize normal vector.")
      .def("__repr__", [](const mrpt::math::TPlane& pl) { return pl.asString(); })
      .def("__str__", [](const mrpt::math::TPlane& pl) { return pl.asString(); });

  // -------------------------------------------------------------------------
  // TBoundingBox (double)
  // -------------------------------------------------------------------------
  py::class_<mrpt::math::TBoundingBox>(
      m, "TBoundingBox", "A 3D axis-aligned bounding box, defined by its min and max corners.")
      .def(
          py::init<const mrpt::math::TPoint3D&, const mrpt::math::TPoint3D&>(),
          "Builds the box from its min and max corners.")
      .def_static(
          "PlusMinusInfinity", &mrpt::math::TBoundingBox::PlusMinusInfinity,
          "Initialize with min=+Infinity, max=-Infinity. This is useful as an initial value before "
          "processing a list of points to keep their minimum/maximum.")
      .def_readwrite("min", &mrpt::math::TBoundingBox::min)
      .def_readwrite("max", &mrpt::math::TBoundingBox::max)
      .def("volume", &mrpt::math::TBoundingBox::volume, "Returns the volume of the box.")
      .def(
          "containsPoint", &mrpt::math::TBoundingBox::containsPoint,
          "Returns true if the point lies within the bounding box (including the exact border)")
      .def(
          "unionWith", &mrpt::math::TBoundingBox::unionWith,
          "Returns the union of this bounding box with \"b\", i.e. a new bounding box comprising "
          "both this and b.")
      .def(
          "intersection",
          [](const mrpt::math::TBoundingBox& self, const mrpt::math::TBoundingBox& other)
          {
            auto result = self.intersection(other);
            if (result.has_value())
            {
              return py::cast(*result);
            }
            return py::none().cast<py::object>();
          },
          "Returns the intersection of this bounding box with \"b\", or std::nullopt if no "
          "intersection exists.")
      .def(
          "__repr__", [](const mrpt::math::TBoundingBox& b)
          { return "TBoundingBox(min=" + b.min.asString() + ", max=" + b.max.asString() + ")"; });

  // -------------------------------------------------------------------------
  // TBoundingBoxf (float)
  // -------------------------------------------------------------------------
  py::class_<mrpt::math::TBoundingBoxf>(
      m, "TBoundingBoxf",
      "A 3D axis-aligned bounding box with float coordinates, defined by its min and max corners.")
      .def(
          py::init<const mrpt::math::TPoint3Df&, const mrpt::math::TPoint3Df&>(),
          "Builds the box from its min and max corners.")
      .def_static(
          "PlusMinusInfinity", &mrpt::math::TBoundingBoxf::PlusMinusInfinity,
          "Initialize with min=+Infinity, max=-Infinity. This is useful as an initial value before "
          "processing a list of points to keep their minimum/maximum.")
      .def_readwrite("min", &mrpt::math::TBoundingBoxf::min)
      .def_readwrite("max", &mrpt::math::TBoundingBoxf::max)
      .def("volume", &mrpt::math::TBoundingBoxf::volume, "Returns the volume of the box.")
      .def(
          "containsPoint", &mrpt::math::TBoundingBoxf::containsPoint,
          "Returns true if the point lies within the bounding box (including the exact border)")
      .def(
          "unionWith", &mrpt::math::TBoundingBoxf::unionWith,
          "Returns the union of this bounding box with \"b\", i.e. a new bounding box comprising "
          "both this and b.")
      .def(
          "__repr__", [](const mrpt::math::TBoundingBoxf& b)
          { return "TBoundingBoxf(min=" + b.min.asString() + ", max=" + b.max.asString() + ")"; });

  // -------------------------------------------------------------------------
  // TTwist2D
  // -------------------------------------------------------------------------
  py::class_<mrpt::math::TTwist2D>(
      m, "TTwist2D", "2D twist: 2D velocity vector (vx,vy) + planar angular velocity (omega)")
      .def(py::init<>(), "Default fast constructor. Initializes to zeros.")
      .def(
          py::init<double, double, double>(), py::arg("vx"), py::arg("vy"), py::arg("omega"),
          "Constructor from components.")
      .def_readwrite("vx", &mrpt::math::TTwist2D::vx)
      .def_readwrite("vy", &mrpt::math::TTwist2D::vy)
      .def_readwrite("omega", &mrpt::math::TTwist2D::omega)
      .def(
          "rotate", &mrpt::math::TTwist2D::rotate,
          "Transform the (vx,vy) components for a counterclockwise rotation of ang radians.")
      .def(
          "rotated", &mrpt::math::TTwist2D::rotated,
          "Like rotate(), but returning a copy of the rotated twist.")
      .def(
          "asString", [](const mrpt::math::TTwist2D& t) { return t.asString(); },
          "Returns a human-readable textual representation of the object (eg: \"[vx vy omega]\", "
          "omega in deg/s)")
      .def(
          "__array__",
          [](const mrpt::math::TTwist2D& t, py::object /*dtype*/, py::object /*copy*/)
          { return py::array_t<double>({3}, {sizeof(double)}, &t.vx); },
          py::arg("dtype") = py::none(), py::arg("copy") = py::none())
      .def("__len__", [](const mrpt::math::TTwist2D&) { return 3; })
      .def("__repr__", [](const mrpt::math::TTwist2D& t) { return t.asString(); })
      .def("__str__", [](const mrpt::math::TTwist2D& t) { return t.asString(); });

  // -------------------------------------------------------------------------
  // TTwist3D
  // -------------------------------------------------------------------------
  py::class_<mrpt::math::TTwist3D>(
      m, "TTwist3D", "3D twist: 3D velocity vector (vx,vy,vz) + angular velocity (wx,wy,wz)")
      .def(py::init<>(), "Default fast constructor. Initializes to zeros.")
      .def(
          py::init<double, double, double, double, double, double>(), py::arg("vx"), py::arg("vy"),
          py::arg("vz"), py::arg("wx"), py::arg("wy"), py::arg("wz"),
          "Constructor from components.")
      .def_readwrite("vx", &mrpt::math::TTwist3D::vx)
      .def_readwrite("vy", &mrpt::math::TTwist3D::vy)
      .def_readwrite("vz", &mrpt::math::TTwist3D::vz)
      .def_readwrite("wx", &mrpt::math::TTwist3D::wx)
      .def_readwrite("wy", &mrpt::math::TTwist3D::wy)
      .def_readwrite("wz", &mrpt::math::TTwist3D::wz)
      .def(
          "rotate", &mrpt::math::TTwist3D::rotate,
          "Transform all 6 components for a change of reference frame from \"A\" to another frame "
          "\"B\" whose rotation with respect to \"A\" is given by rot.")
      .def(
          "rotated", &mrpt::math::TTwist3D::rotated,
          "Like rotate(), but returning a copy of the rotated twist.")
      .def(
          "asString", [](const mrpt::math::TTwist3D& t) { return t.asString(); },
          "Returns a human-readable textual representation of the object (eg: \"[vx vy vz wx wy "
          "wz]\", omegas in deg/s)")
      .def(
          "__array__",
          [](const mrpt::math::TTwist3D& t, py::object /*dtype*/, py::object /*copy*/)
          { return py::array_t<double>({6}, {sizeof(double)}, &t.vx); },
          py::arg("dtype") = py::none(), py::arg("copy") = py::none())
      .def("__len__", [](const mrpt::math::TTwist3D&) { return 6; })
      .def("__repr__", [](const mrpt::math::TTwist3D& t) { return t.asString(); })
      .def("__str__", [](const mrpt::math::TTwist3D& t) { return t.asString(); });

  // -------------------------------------------------------------------------
  // TPose3DQuat
  // -------------------------------------------------------------------------
  py::class_<mrpt::math::TPose3DQuat>(
      m, "TPose3DQuat", "Lightweight 3D pose (three spatial coordinates, plus a quaternion ).")
      .def(py::init<>(), "Default fast constructor. Initializes to identity transformation.")
      .def(
          py::init<double, double, double, double, double, double, double>(), py::arg("x"),
          py::arg("y"), py::arg("z"), py::arg("qr"), py::arg("qx"), py::arg("qy"), py::arg("qz"),
          "Constructor from coordinates.")
      .def_readwrite("x", &mrpt::math::TPose3DQuat::x)
      .def_readwrite("y", &mrpt::math::TPose3DQuat::y)
      .def_readwrite("z", &mrpt::math::TPose3DQuat::z)
      .def_readwrite("qr", &mrpt::math::TPose3DQuat::qr)
      .def_readwrite("qx", &mrpt::math::TPose3DQuat::qx)
      .def_readwrite("qy", &mrpt::math::TPose3DQuat::qy)
      .def_readwrite("qz", &mrpt::math::TPose3DQuat::qz)
      .def(
          "__array__",
          [](const mrpt::math::TPose3DQuat& p, py::object /*dtype*/, py::object /*copy*/)
          { return py::array_t<double>({7}, {sizeof(double)}, &p.x); },
          py::arg("dtype") = py::none(), py::arg("copy") = py::none())
      .def("__len__", [](const mrpt::math::TPose3DQuat&) { return 7; })
      .def("__repr__", [](const mrpt::math::TPose3DQuat& p) { return p.asString(); })
      .def("__str__", [](const mrpt::math::TPose3DQuat& p) { return p.asString(); });

  // -------------------------------------------------------------------------
  // CPolygon
  // -------------------------------------------------------------------------
  py::class_<
      mrpt::math::CPolygon, mrpt::serialization::CSerializable,
      std::shared_ptr<mrpt::math::CPolygon>>(m, "CPolygon", "A 2D polygon, serializable.")
      .def(py::init<>(), "Default constructor (empty polygon, 0 vertices)")
      .def(
          "add_vertex", &mrpt::math::CPolygon::add_vertex, py::arg("x"), py::arg("y"),
          "Add a new vertex to polygon.")
      .def(
          "get_vertex_x", &mrpt::math::CPolygon::get_vertex_x,
          "Returns the x coordinate of the i-th vertex.")
      .def(
          "get_vertex_y", &mrpt::math::CPolygon::get_vertex_y,
          "Returns the y coordinate of the i-th vertex.")
      .def(
          "get_vertices",
          [](const mrpt::math::CPolygon& self)
          {
            std::vector<double> xs;
            std::vector<double> ys;
            self.get_vertices(xs, ys);
            return py::make_tuple(xs, ys);
          },
          "Returns (xs, ys) as two lists of vertex coordinates")
      .def(
          "set_vertices",
          [](mrpt::math::CPolygon& self, const std::vector<double>& xs,
             const std::vector<double>& ys) { self.set_vertices(xs, ys); },
          "Set all vertices at once.")
      .def("__len__", [](const mrpt::math::CPolygon& self) { return self.size(); })
      .def(
          "__repr__", [](const mrpt::math::CPolygon& self)
          { return "CPolygon(" + std::to_string(self.size()) + " vertices)"; });

  // -------------------------------------------------------------------------
  // CHistogram
  // -------------------------------------------------------------------------
  py::class_<mrpt::math::CHistogram, std::shared_ptr<mrpt::math::CHistogram>>(
      m, "CHistogram", "A histogram of a real-valued variable, with equally-sized bins.")
      .def(
          py::init<double, double, size_t>(), py::arg("min"), py::arg("max"), py::arg("nBins"),
          "Constructor.")
      .def("clear", &mrpt::math::CHistogram::clear, "Resets all bins to zero.")
      .def(
          "add",
          static_cast<void (mrpt::math::CHistogram::*)(double)>(&mrpt::math::CHistogram::add),
          "Add an element to the histogram. If element is out of [min,max] it is ignored.")
      .def(
          "getBinCount", &mrpt::math::CHistogram::getBinCount,
          "Returns the elements count into the selected bin index, where first one is 0.")
      .def(
          "getBinRatio", &mrpt::math::CHistogram::getBinRatio,
          "Returns the ratio in [0,1] range for the selected bin index, where first one is 0.")
      .def(
          "getHistogram",
          [](const mrpt::math::CHistogram& self)
          {
            std::vector<double> x;
            std::vector<double> hits;
            self.getHistogram(x, hits);
            return py::make_tuple(x, hits);
          },
          "Returns (bin_centers, hit_counts) as two lists")
      .def(
          "getHistogramNormalized",
          [](const mrpt::math::CHistogram& self)
          {
            std::vector<double> x;
            std::vector<double> hits;
            self.getHistogramNormalized(x, hits);
            return py::make_tuple(x, hits);
          },
          "Returns (bin_centers, normalized_hits) — integral equals 1 as PDF")
      .def(
          "__repr__",
          []([[maybe_unused]] const mrpt::math::CHistogram& self) { return "CHistogram()"; });

  // =========================================================================
  // Free math functions
  // =========================================================================
  m.def(
      "wrapToPi", [](double a) { return mrpt::math::wrapToPi(a); }, "Wrap angle to [-pi, pi]");
  m.def(
      "wrapTo2Pi", [](double a) { return mrpt::math::wrapTo2Pi(a); }, "Wrap angle to [0, 2*pi]");
}
