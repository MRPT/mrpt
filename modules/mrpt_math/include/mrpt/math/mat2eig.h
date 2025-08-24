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

#include <mrpt/core/is_defined.h>
#include <mrpt/math/math_frwds.h>

namespace mrpt::math
{
/** Returns an Eigen-compatible type, despite its argument already is an Eigen
 * matrix, or an mrpt-math matrix/vector. \ingroup mrpt_math_grp
 */
template <class Derived>
const Derived& mat2eig(const Eigen::EigenBase<Derived>& m)
{
  return m.derived();
}

template <class MAT>
auto mat2eig(const MAT& m, typename MAT::eigen_t* = nullptr)
{
  return m.asEigen();
}

}  // namespace mrpt::math
