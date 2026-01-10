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
#pragma once

#include <mrpt/math/CMatrixFixed.h>

#include <Eigen/Dense>

namespace mrpt::math
{
template <typename T, matrix_dim_t ROWS, matrix_dim_t COLS>
CMatrixFixed<float, ROWS, COLS> CMatrixFixed<T, ROWS, COLS>::cast_float() const
{
  CMatrixFixed<float, ROWS, COLS> r(rows(), cols());
  r.asEigen() = asEigen().template cast<float>();
  return r;
}

template <typename T, matrix_dim_t ROWS, matrix_dim_t COLS>
CMatrixFixed<double, ROWS, COLS> CMatrixFixed<T, ROWS, COLS>::cast_double() const
{
  CMatrixFixed<double, ROWS, COLS> r(rows(), cols());
  r.asEigen() = asEigen().template cast<double>();
  return r;
}

template <typename T, matrix_dim_t ROWS, matrix_dim_t COLS>
CMatrixFixed<T, ROWS, 1> CMatrixFixed<T, ROWS, COLS>::llt_solve(
    const CMatrixFixed<T, ROWS, 1>& b) const
{
  if constexpr (ROWS == COLS)
  {
    auto ret = CMatrixFixed<T, ROWS, 1>(asEigen().llt().solve(b.asEigen()));
    return ret;
  }
  else
  {
    throw std::invalid_argument("llt_solve(): only available for square matrices.");
  }
}
template <typename T, matrix_dim_t ROWS, matrix_dim_t COLS>
CMatrixFixed<T, ROWS, 1> CMatrixFixed<T, ROWS, COLS>::lu_solve(
    const CMatrixFixed<T, ROWS, 1>& b) const
{
  if constexpr (ROWS == COLS)
  {
    auto ret = CMatrixFixed<T, ROWS, 1>(asEigen().lu().solve(b.asEigen()));
    return ret;
  }
  else
  {
    throw std::invalid_argument("lu_solve(): only available for square matrices.");
  }
}

}  // namespace mrpt::math
