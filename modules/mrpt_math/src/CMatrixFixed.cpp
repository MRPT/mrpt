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

#include <mrpt/math/CMatrixFixed.h>

#include "CMatrixFixed_impl.h"

// See also: "MatrixBase_*.cpp" and "MatrixVectorBase_*.cpp"
// for explicit instantiations of the corresponding base classes.

using namespace mrpt::math;

// Template instantiations:
#define DO_MATFIXED_INSTANTIATION_NM(T_, N_, M_) \
  template class mrpt::math::CMatrixFixed<T_, N_, M_>;

#define DO_MATFIXED_INSTANTIATION(T_)    \
  DO_MATFIXED_INSTANTIATION_NM(T_, 2, 2) \
  DO_MATFIXED_INSTANTIATION_NM(T_, 3, 3) \
  DO_MATFIXED_INSTANTIATION_NM(T_, 4, 4) \
  DO_MATFIXED_INSTANTIATION_NM(T_, 6, 6) \
  DO_MATFIXED_INSTANTIATION_NM(T_, 7, 7)

DO_MATFIXED_INSTANTIATION(float);
DO_MATFIXED_INSTANTIATION(double);
