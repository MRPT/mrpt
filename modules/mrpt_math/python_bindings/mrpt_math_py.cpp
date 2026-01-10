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
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/CVectorFixed.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/serialization/CSerializable.h>

namespace py = pybind11;

// Define RowMajor Eigen types to match MRPT's internal layout
// This facilitates zero-copy mapping to NumPy
using EigenRowMatrixXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using EigenRowVectorXd = Eigen::Matrix<double, 1, Eigen::Dynamic, Eigen::RowMajor>;

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
}