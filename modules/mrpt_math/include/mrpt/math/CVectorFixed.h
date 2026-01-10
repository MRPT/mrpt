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
#include <mrpt/typemeta/TTypeName.h>
#include <mrpt/typemeta/num_to_string.h>

namespace mrpt
{
namespace math
{
/** CVectorFixed is an array for numeric types supporting several mathematical
 * operations (actually, just a wrapper on Eigen::Matrix<T,N,1>)
 * \sa CVectorFixedFloat, CVectorFixedDouble, CArray
 */
template <typename T, matrix_dim_t N>
using CVectorFixed = CMatrixFixed<T, N, 1>;

/** Specialization of CVectorFixed for float numbers \sa CVectorFixed  */
template <matrix_dim_t N>
using CVectorFixedFloat = CVectorFixed<float, N>;

/** Specialization of CVectorFixed for double numbers \sa CVectorFixed  */
template <matrix_dim_t N>
using CVectorFixedDouble = CVectorFixed<double, N>;

}  // namespace math

namespace typemeta
{
// Extensions to mrpt::typemeta::TTypeName for matrices:
template <typename T, mrpt::math::matrix_dim_t N>
struct TTypeName<mrpt::math::CVectorFixed<T, N>>
{
  constexpr static auto get()
  {
    return literal("CVectorFixed<") + TTypeName<T>::get() + literal(",") +
           literal(num_to_string<N>::value) + literal(">");
  }
};
template <mrpt::math::matrix_dim_t N>
struct TTypeName<mrpt::math::CVectorFixedDouble<N>>
{
  constexpr static auto get()
  {
    return literal("CVectorFixedDouble<") + literal(num_to_string<N>::value) + literal(">");
  }
};
template <mrpt::math::matrix_dim_t N>
struct TTypeName<mrpt::math::CVectorFixedFloat<N>>
{
  constexpr static auto get()
  {
    return literal("CVectorFixedFloat<") + literal(num_to_string<N>::value) + literal(">");
  }
};
}  // namespace typemeta
}  // namespace mrpt
