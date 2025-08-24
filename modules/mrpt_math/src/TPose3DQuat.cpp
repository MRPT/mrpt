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

#include "math-precomp.h"  // Precompiled headers
//
#include <mrpt/core/bits_math.h>  // square
#include <mrpt/math/TPose3DQuat.h>

#include <cmath>

using namespace mrpt::math;

static_assert(std::is_trivially_copyable_v<TPose3DQuat>);

void TPose3DQuat::fromString(const std::string& s)
{
  CMatrixDouble m;
  if (!m.fromMatlabStringFormat(s))
    THROW_EXCEPTION_FMT("Malformed expression in ::fromString, s=\"%s\"", s.c_str());
  ASSERTMSG_(m.rows() == 1 && m.cols() == 7, "Wrong size of vector in ::fromString");
  for (int i = 0; i < m.cols(); i++) (*this)[i] = m(0, i);
}

double TPose3DQuat::norm() const
{
  return std::sqrt(mrpt::square(x) + mrpt::square(y) + mrpt::square(z));
}
