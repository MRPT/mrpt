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
#pragma once

#include <mrpt/math/math_frwds.h>

#include <array>

namespace mrpt::math
{
/** Auxiliary class used in CMatrixDynamic:size(), CMatrixDynamic::resize(),
 * CMatrixFixed::size(), CMatrixFixed::resize(), to mimic the
 * behavior of STL-containers.
 * \ingroup mrpt_math_grp
 */
struct matrix_size_t : public std::array<matrix_dim_t, 2>
{
  constexpr matrix_size_t() : std::array<matrix_dim_t, 2>{0, 0} {}
  constexpr matrix_size_t(const matrix_dim_t rows, const matrix_dim_t cols) :
      std::array<matrix_dim_t, 2>{rows, cols}
  {
  }

  /** Cast to size_t as the overall number of matrix/vector elements */
  operator matrix_dim_t() const { return at(0) * at(1); }
};

}  // namespace mrpt::math
