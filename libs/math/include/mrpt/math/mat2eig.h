/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
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
