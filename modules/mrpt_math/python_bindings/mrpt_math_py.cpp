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
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

// MRPT math headers
#include <mrpt/math/CHistogram.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CPolygon.h>
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
  py::class_<T, std::shared_ptr<T>>(m, name.c_str())
      .def(py::init<>())
      .def("as_numpy", [](const T& self) { return self.asEigen(); })
      .def("__array__", [](const T& self) { return self.asEigen(); })
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
      m, "CMatrixDouble")
      .def(py::init<>())
      .def(py::init<int, int>())
      // Automatic conversion: NumPy -> CMatrixDouble
      .def(py::init(
          [](const EigenRowMatrixXd& src)
          {
            auto m = std::make_shared<mrpt::math::CMatrixDouble>();
            m->resize(src.rows(), src.cols());
            for (int r = 0; r < src.rows(); ++r)
            {
              for (int c = 0; c < src.cols(); ++c)
              {
                (*m)(r, c) = src(r, c);
              }
            }
            return m;
          }))
      // Manual conversion: mrpt_obj.as_numpy()
      .def(
          "as_numpy",
          [](const mrpt::math::CMatrixDouble& self)
          {
            // Return an Eigen Map that NumPy can consume directly
            return Eigen::Map<const EigenRowMatrixXd>(&self(0, 0), self.rows(), self.cols());
          })
      .def("__repr__", [](const mrpt::math::CMatrixDouble& self) { return self.asString(); });

  // -------------------------------------------------------------------------
  // 2. CVectorDouble (Dynamic size)
  // -------------------------------------------------------------------------
  py::class_<mrpt::math::CVectorDouble, std::shared_ptr<mrpt::math::CVectorDouble>>(
      m, "CVectorDouble")
      .def(py::init<>())
      .def(py::init<size_t>())
      .def(py::init(
          [](const EigenRowVectorXd& src)
          {
            auto v = std::make_shared<mrpt::math::CVectorDouble>();
            v->resize(src.size());
            for (int i = 0; i < src.size(); ++i)
            {
              (*v)[i] = src[i];
            }
            return v;
          }))
      .def("as_numpy", [](const mrpt::math::CVectorDouble& self) { return self.asEigen(); });

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
  // Lightweight pose types
  // -------------------------------------------------------------------------
  // -------------------------------------------------------------------------
  // TPoint2D
  // -------------------------------------------------------------------------
  py::class_<mrpt::math::TPoint2D, std::shared_ptr<mrpt::math::TPoint2D>>(m, "TPoint2D")
      .def(py::init<>())
      .def(py::init<double, double>())
      .def(py::init(
          [](const std::vector<double>& v)
          {
            if (v.size() != 2)
            {
              throw std::invalid_argument("List must have 2 elements [x, y]");
            }
            return std::make_shared<mrpt::math::TPoint2D>(v[0], v[1]);
          }))
      .def_readwrite("x", &mrpt::math::TPoint2D::x)
      .def_readwrite("y", &mrpt::math::TPoint2D::y)
      .def("cast_float", [](const mrpt::math::TPoint2D& p) { return p.cast<float>(); })
      .def("__repr__", [](const mrpt::math::TPoint2D& p) { return p.asString(); });

  // -------------------------------------------------------------------------
  // TPoint3D
  // -------------------------------------------------------------------------
  py::class_<mrpt::math::TPoint3D, std::shared_ptr<mrpt::math::TPoint3D>>(m, "TPoint3D")
      .def(py::init<>())
      .def(py::init<double, double, double>())
      .def(py::init(
          [](const std::vector<double>& v)
          {
            if (v.size() != 3)
            {
              throw std::invalid_argument("List must have 3 elements [x, y, z]");
            }
            return std::make_shared<mrpt::math::TPoint3D>(v[0], v[1], v[2]);
          }))
      .def_readwrite("x", &mrpt::math::TPoint3D::x)
      .def_readwrite("y", &mrpt::math::TPoint3D::y)
      .def_readwrite("z", &mrpt::math::TPoint3D::z)
      .def("cast_float", [](const mrpt::math::TPoint3D& p) { return p.cast<float>(); })
      .def("__repr__", [](const mrpt::math::TPoint3D& p) { return p.asString(); });

  // -------------------------------------------------------------------------
  // TPoint2Df
  // -------------------------------------------------------------------------
  py::class_<mrpt::math::TPoint2Df, std::shared_ptr<mrpt::math::TPoint2Df>>(m, "TPoint2Df")
      .def(py::init<>())
      .def(py::init<float, float>())
      .def(py::init(
          [](const std::vector<float>& v)
          {
            if (v.size() != 2)
            {
              throw std::invalid_argument("List must have 2 elements [x, y]");
            }
            return std::make_shared<mrpt::math::TPoint2Df>(v[0], v[1]);
          }))
      .def_readwrite("x", &mrpt::math::TPoint2Df::x)
      .def_readwrite("y", &mrpt::math::TPoint2Df::y)
      .def("cast_double", [](const mrpt::math::TPoint2Df& p) { return p.cast<double>(); })
      .def("__repr__", [](const mrpt::math::TPoint2Df& p) { return p.asString(); });

  // -------------------------------------------------------------------------
  // TPoint3Df
  // -------------------------------------------------------------------------
  py::class_<mrpt::math::TPoint3Df, std::shared_ptr<mrpt::math::TPoint3Df>>(m, "TPoint3Df")
      .def(py::init<>())
      .def(py::init<float, float, float>())
      .def(py::init(
          [](const std::vector<float>& v)
          {
            if (v.size() != 3)
            {
              throw std::invalid_argument("List must have 3 elements [x, y, z]");
            }
            return std::make_shared<mrpt::math::TPoint3Df>(v[0], v[1], v[2]);
          }))
      .def_readwrite("x", &mrpt::math::TPoint3Df::x)
      .def_readwrite("y", &mrpt::math::TPoint3Df::y)
      .def_readwrite("z", &mrpt::math::TPoint3Df::z)
      .def("cast_double", [](const mrpt::math::TPoint3Df& p) { return p.cast<double>(); })
      .def("__repr__", [](const mrpt::math::TPoint3Df& p) { return p.asString(); });

  // -------------------------------------------------------------------------
  // TPose2D
  // -------------------------------------------------------------------------
  py::class_<mrpt::math::TPose2D, std::shared_ptr<mrpt::math::TPose2D>>(m, "TPose2D")
      .def(py::init<>())
      .def(py::init<double, double, double>())
      .def(py::init(
          [](const std::vector<double>& v)
          {
            if (v.size() != 3)
            {
              throw std::invalid_argument("List must have 3 elements [x, y, phi]");
            }
            return std::make_shared<mrpt::math::TPose2D>(v[0], v[1], v[2]);
          }))
      .def_readwrite("x", &mrpt::math::TPose2D::x)
      .def_readwrite("y", &mrpt::math::TPose2D::y)
      .def_readwrite("phi", &mrpt::math::TPose2D::phi)
      .def("__repr__", [](const mrpt::math::TPose2D& p) { return p.asString(); });

  // -------------------------------------------------------------------------
  // TPose3D
  // -------------------------------------------------------------------------
  py::class_<mrpt::math::TPose3D, std::shared_ptr<mrpt::math::TPose3D>>(m, "TPose3D")
      .def(py::init<>())
      .def(py::init<double, double, double, double, double, double>())
      .def(py::init(
          [](const std::vector<double>& v)
          {
            if (v.size() != 6)
            {
              throw std::invalid_argument("List must have 6 elements [x, y, z, yaw, pitch, roll]");
            }
            return std::make_shared<mrpt::math::TPose3D>(v[0], v[1], v[2], v[3], v[4], v[5]);
          }))
      .def_readwrite("x", &mrpt::math::TPose3D::x)
      .def_readwrite("y", &mrpt::math::TPose3D::y)
      .def_readwrite("z", &mrpt::math::TPose3D::z)
      .def_readwrite("yaw", &mrpt::math::TPose3D::yaw)
      .def_readwrite("pitch", &mrpt::math::TPose3D::pitch)
      .def_readwrite("roll", &mrpt::math::TPose3D::roll)
      .def("__repr__", [](const mrpt::math::TPose3D& p) { return p.asString(); });

  // =========================================================================
  // Geometry primitives (Phase 0.1 extensions)
  // =========================================================================

  // -------------------------------------------------------------------------
  // TSegment2D
  // -------------------------------------------------------------------------
  py::class_<mrpt::math::TSegment2D, std::shared_ptr<mrpt::math::TSegment2D>>(m, "TSegment2D")
      .def(py::init<>())
      .def(py::init<const mrpt::math::TPoint2D&, const mrpt::math::TPoint2D&>())
      .def_static(
          "FromPoints", [](const mrpt::math::TPoint2D& p1, const mrpt::math::TPoint2D& p2)
          { return mrpt::math::TSegment2D::FromPoints(p1, p2); })
      .def_readwrite("point1", &mrpt::math::TSegment2D::point1)
      .def_readwrite("point2", &mrpt::math::TSegment2D::point2)
      .def("length", &mrpt::math::TSegment2D::length)
      .def(
          "distance", py::overload_cast<const mrpt::math::TPoint2D&>(
                          &mrpt::math::TSegment2D::distance, py::const_))
      .def("contains", &mrpt::math::TSegment2D::contains)
      .def("__repr__", [](const mrpt::math::TSegment2D& s) { return s.asString(); });

  // -------------------------------------------------------------------------
  // TSegment3D
  // -------------------------------------------------------------------------
  py::class_<mrpt::math::TSegment3D, std::shared_ptr<mrpt::math::TSegment3D>>(m, "TSegment3D")
      .def(py::init<>())
      .def(py::init<const mrpt::math::TPoint3D&, const mrpt::math::TPoint3D&>())
      .def_readwrite("point1", &mrpt::math::TSegment3D::point1)
      .def_readwrite("point2", &mrpt::math::TSegment3D::point2)
      .def("length", &mrpt::math::TSegment3D::length)
      .def(
          "distance", py::overload_cast<const mrpt::math::TPoint3D&>(
                          &mrpt::math::TSegment3D::distance, py::const_))
      .def("contains", &mrpt::math::TSegment3D::contains)
      .def("__repr__", [](const mrpt::math::TSegment3D& s) { return s.asString(); });

  // -------------------------------------------------------------------------
  // TLine2D
  // -------------------------------------------------------------------------
  py::class_<mrpt::math::TLine2D, std::shared_ptr<mrpt::math::TLine2D>>(m, "TLine2D")
      .def(py::init<>())
      .def(py::init<const mrpt::math::TPoint2D&, const mrpt::math::TPoint2D&>())
      .def(py::init<double, double, double>(), py::arg("A"), py::arg("B"), py::arg("C"))
      .def_static(
          "FromTwoPoints", [](const mrpt::math::TPoint2D& p1, const mrpt::math::TPoint2D& p2)
          { return mrpt::math::TLine2D::FromTwoPoints(p1, p2); })
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
      .def("evaluatePoint", &mrpt::math::TLine2D::evaluatePoint)
      .def("contains", &mrpt::math::TLine2D::contains)
      .def("distance", &mrpt::math::TLine2D::distance)
      .def("unitarize", &mrpt::math::TLine2D::unitarize)
      .def("__repr__", [](const mrpt::math::TLine2D& l) { return l.asString(); })
      .def("__str__", [](const mrpt::math::TLine2D& l) { return l.asString(); });

  // -------------------------------------------------------------------------
  // TLine3D
  // -------------------------------------------------------------------------
  py::class_<mrpt::math::TLine3D, std::shared_ptr<mrpt::math::TLine3D>>(m, "TLine3D")
      .def(py::init<>())
      .def(py::init<const mrpt::math::TPoint3D&, const mrpt::math::TPoint3D&>())
      .def_static(
          "FromTwoPoints", [](const mrpt::math::TPoint3D& p1, const mrpt::math::TPoint3D& p2)
          { return mrpt::math::TLine3D::FromTwoPoints(p1, p2); })
      .def_readwrite("pBase", &mrpt::math::TLine3D::pBase)
      .def_readwrite("director", &mrpt::math::TLine3D::director)
      .def("contains", &mrpt::math::TLine3D::contains)
      .def(
          "distance", py::overload_cast<const mrpt::math::TPoint3D&>(
                          &mrpt::math::TLine3D::distance, py::const_))
      .def("unitarize", &mrpt::math::TLine3D::unitarize)
      .def("closestPointTo", &mrpt::math::TLine3D::closestPointTo)
      .def("__repr__", [](const mrpt::math::TLine3D& l) { return l.asString(); })
      .def("__str__", [](const mrpt::math::TLine3D& l) { return l.asString(); });

  // -------------------------------------------------------------------------
  // TPlane
  // -------------------------------------------------------------------------
  py::class_<mrpt::math::TPlane, std::shared_ptr<mrpt::math::TPlane>>(m, "TPlane")
      .def(py::init<>())
      .def(
          py::init<double, double, double, double>(), py::arg("A"), py::arg("B"), py::arg("C"),
          py::arg("D"))
      .def(py::init<
           const mrpt::math::TPoint3D&, const mrpt::math::TPoint3D&, const mrpt::math::TPoint3D&>())
      .def_static(
          "From3Points", [](const mrpt::math::TPoint3D& p1, const mrpt::math::TPoint3D& p2,
                            const mrpt::math::TPoint3D& p3)
          { return mrpt::math::TPlane::From3Points(p1, p2, p3); })
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
      .def("evaluatePoint", &mrpt::math::TPlane::evaluatePoint)
      .def(
          "contains",
          py::overload_cast<const mrpt::math::TPoint3D&>(&mrpt::math::TPlane::contains, py::const_))
      .def(
          "distance",
          py::overload_cast<const mrpt::math::TPoint3D&>(&mrpt::math::TPlane::distance, py::const_))
      .def("unitarize", &mrpt::math::TPlane::unitarize)
      .def("__repr__", [](const mrpt::math::TPlane& pl) { return pl.asString(); })
      .def("__str__", [](const mrpt::math::TPlane& pl) { return pl.asString(); });

  // -------------------------------------------------------------------------
  // TBoundingBox (double)
  // -------------------------------------------------------------------------
  py::class_<mrpt::math::TBoundingBox, std::shared_ptr<mrpt::math::TBoundingBox>>(m, "TBoundingBox")
      .def(py::init<const mrpt::math::TPoint3D&, const mrpt::math::TPoint3D&>())
      .def_static("PlusMinusInfinity", &mrpt::math::TBoundingBox::PlusMinusInfinity)
      .def_readwrite("min", &mrpt::math::TBoundingBox::min)
      .def_readwrite("max", &mrpt::math::TBoundingBox::max)
      .def("volume", &mrpt::math::TBoundingBox::volume)
      .def("containsPoint", &mrpt::math::TBoundingBox::containsPoint)
      .def("unionWith", &mrpt::math::TBoundingBox::unionWith)
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
          })
      .def(
          "__repr__", [](const mrpt::math::TBoundingBox& b)
          { return "TBoundingBox(min=" + b.min.asString() + ", max=" + b.max.asString() + ")"; });

  // -------------------------------------------------------------------------
  // TBoundingBoxf (float)
  // -------------------------------------------------------------------------
  py::class_<mrpt::math::TBoundingBoxf, std::shared_ptr<mrpt::math::TBoundingBoxf>>(
      m, "TBoundingBoxf")
      .def(py::init<const mrpt::math::TPoint3Df&, const mrpt::math::TPoint3Df&>())
      .def_static("PlusMinusInfinity", &mrpt::math::TBoundingBoxf::PlusMinusInfinity)
      .def_readwrite("min", &mrpt::math::TBoundingBoxf::min)
      .def_readwrite("max", &mrpt::math::TBoundingBoxf::max)
      .def("volume", &mrpt::math::TBoundingBoxf::volume)
      .def("containsPoint", &mrpt::math::TBoundingBoxf::containsPoint)
      .def("unionWith", &mrpt::math::TBoundingBoxf::unionWith)
      .def(
          "__repr__", [](const mrpt::math::TBoundingBoxf& b)
          { return "TBoundingBoxf(min=" + b.min.asString() + ", max=" + b.max.asString() + ")"; });

  // -------------------------------------------------------------------------
  // TTwist2D
  // -------------------------------------------------------------------------
  py::class_<mrpt::math::TTwist2D, std::shared_ptr<mrpt::math::TTwist2D>>(m, "TTwist2D")
      .def(py::init<>())
      .def(py::init<double, double, double>(), py::arg("vx"), py::arg("vy"), py::arg("omega"))
      .def_readwrite("vx", &mrpt::math::TTwist2D::vx)
      .def_readwrite("vy", &mrpt::math::TTwist2D::vy)
      .def_readwrite("omega", &mrpt::math::TTwist2D::omega)
      .def("rotate", &mrpt::math::TTwist2D::rotate)
      .def("rotated", &mrpt::math::TTwist2D::rotated)
      .def("asString", py::overload_cast<>(&mrpt::math::TTwist2D::asString, py::const_))
      .def("__repr__", [](const mrpt::math::TTwist2D& t) { return t.asString(); })
      .def("__str__", [](const mrpt::math::TTwist2D& t) { return t.asString(); });

  // -------------------------------------------------------------------------
  // TTwist3D
  // -------------------------------------------------------------------------
  py::class_<mrpt::math::TTwist3D, std::shared_ptr<mrpt::math::TTwist3D>>(m, "TTwist3D")
      .def(py::init<>())
      .def(
          py::init<double, double, double, double, double, double>(), py::arg("vx"), py::arg("vy"),
          py::arg("vz"), py::arg("wx"), py::arg("wy"), py::arg("wz"))
      .def_readwrite("vx", &mrpt::math::TTwist3D::vx)
      .def_readwrite("vy", &mrpt::math::TTwist3D::vy)
      .def_readwrite("vz", &mrpt::math::TTwist3D::vz)
      .def_readwrite("wx", &mrpt::math::TTwist3D::wx)
      .def_readwrite("wy", &mrpt::math::TTwist3D::wy)
      .def_readwrite("wz", &mrpt::math::TTwist3D::wz)
      .def("rotate", &mrpt::math::TTwist3D::rotate)
      .def("rotated", &mrpt::math::TTwist3D::rotated)
      .def("asString", py::overload_cast<>(&mrpt::math::TTwist3D::asString, py::const_))
      .def("__repr__", [](const mrpt::math::TTwist3D& t) { return t.asString(); })
      .def("__str__", [](const mrpt::math::TTwist3D& t) { return t.asString(); });

  // -------------------------------------------------------------------------
  // TPose3DQuat
  // -------------------------------------------------------------------------
  py::class_<mrpt::math::TPose3DQuat, std::shared_ptr<mrpt::math::TPose3DQuat>>(m, "TPose3DQuat")
      .def(py::init<>())
      .def(
          py::init<double, double, double, double, double, double, double>(), py::arg("x"),
          py::arg("y"), py::arg("z"), py::arg("qr"), py::arg("qx"), py::arg("qy"), py::arg("qz"))
      .def_readwrite("x", &mrpt::math::TPose3DQuat::x)
      .def_readwrite("y", &mrpt::math::TPose3DQuat::y)
      .def_readwrite("z", &mrpt::math::TPose3DQuat::z)
      .def_readwrite("qr", &mrpt::math::TPose3DQuat::qr)
      .def_readwrite("qx", &mrpt::math::TPose3DQuat::qx)
      .def_readwrite("qy", &mrpt::math::TPose3DQuat::qy)
      .def_readwrite("qz", &mrpt::math::TPose3DQuat::qz)
      .def("__repr__", [](const mrpt::math::TPose3DQuat& p) { return p.asString(); })
      .def("__str__", [](const mrpt::math::TPose3DQuat& p) { return p.asString(); });

  // -------------------------------------------------------------------------
  // CPolygon
  // -------------------------------------------------------------------------
  py::class_<
      mrpt::math::CPolygon, mrpt::serialization::CSerializable,
      std::shared_ptr<mrpt::math::CPolygon>>(m, "CPolygon")
      .def(py::init<>())
      .def("add_vertex", &mrpt::math::CPolygon::add_vertex, py::arg("x"), py::arg("y"))
      .def("get_vertex_x", &mrpt::math::CPolygon::get_vertex_x)
      .def("get_vertex_y", &mrpt::math::CPolygon::get_vertex_y)
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
          "set_vertices", [](mrpt::math::CPolygon& self, const std::vector<double>& xs,
                             const std::vector<double>& ys) { self.set_vertices(xs, ys); })
      .def("__len__", [](const mrpt::math::CPolygon& self) { return self.size(); })
      .def(
          "__repr__", [](const mrpt::math::CPolygon& self)
          { return "CPolygon(" + std::to_string(self.size()) + " vertices)"; });

  // -------------------------------------------------------------------------
  // CHistogram
  // -------------------------------------------------------------------------
  py::class_<mrpt::math::CHistogram, std::shared_ptr<mrpt::math::CHistogram>>(m, "CHistogram")
      .def(py::init<double, double, size_t>(), py::arg("min"), py::arg("max"), py::arg("nBins"))
      .def("clear", &mrpt::math::CHistogram::clear)
      .def(
          "add",
          static_cast<void (mrpt::math::CHistogram::*)(double)>(&mrpt::math::CHistogram::add))
      .def("getBinCount", &mrpt::math::CHistogram::getBinCount)
      .def("getBinRatio", &mrpt::math::CHistogram::getBinRatio)
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
