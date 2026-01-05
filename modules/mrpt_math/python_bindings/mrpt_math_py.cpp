/*                    _
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
}