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

/** \file Provide helper functions for MEX/MATLAB.
 * This file can be safely included without checking MRPT_HAS_MATLAB
 */

#include <mrpt/core/config.h>  // MRPT_HAS_MATLAB

#if MRPT_HAS_MATLAB
#include <mexplus.h>
#endif
#include <mrpt/math/math_frwds.h>

namespace mrpt::math
{
#if MRPT_HAS_MATLAB
/** \addtogroup matlab_grp Helper functions for MEX & MATLAB
 *  \ingroup mrpt_math_grp
 * @{ */

/** Convert vectors, arrays and matrices into Matlab vectors/matrices.
 * Supported input classes:
 *  - Eigen::Matrix<T,N,1>
 *  - mrpt::math::CVectorFixed<T,N>
 *  - mrpt::math::CMatrixF{*}
 */
template <typename MATRIX>
mxArray* convertToMatlab(const MATRIX& mat)
{
  const size_t m = mat.rows(), n = mat.cols();
  mxArray* mxa = mxCreateDoubleMatrix(m, n, mxREAL);
  // *IMPORTANT* Matlab stores matrices in *column-major* order!
  double* mxa_data = mxGetPr(mxa);
  for (size_t j = 0; j < n; j++)    // column
    for (size_t i = 0; i < m; i++)  // rows
      *mxa_data++ = mat.coeff(i, j);
  return mxa;
}

/** Convert std::vector<> or std::deque<> of numeric types into Matlab vectors
 */
template <typename CONTAINER>
mxArray* convertVectorToMatlab(const CONTAINER& vec)
{
  const size_t m = vec.size(), n = 1;
  mxArray* mxa = mxCreateDoubleMatrix(m, n, mxREAL);
  // *IMPORTANT* Matlab stores matrices in *column-major* order!
  double* mxa_data = mxGetPr(mxa);
  for (size_t i = 0; i < m; i++)  // rows
    *mxa_data++ = vec[i];
  return mxa;
}

/** @} */
#endif
}  // namespace mrpt::math
