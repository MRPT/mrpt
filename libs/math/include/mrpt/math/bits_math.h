/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

namespace mrpt::math
{
// This inline function is used everywhere, so just move it here even it's not a
// forward declaration!
/*! Returns the size of the matrix in the i'th dimension: 1=rows, 2=columns
 * (MATLAB-compatible function)
 *  \note Template argument MATRIXLIKE can be: mrpt::math::CMatrixDynamic,
 * mrpt::math::CMatrixDynamic, mrpt::math::CMatrixFixed
 */
template <class MATRIXLIKE>
inline size_t size(const MATRIXLIKE& m, const int dim)
{
	if (dim == 1)
		return m.rows();
	else if (dim == 2)
		return m.cols();
	else
		THROW_EXCEPTION_FMT(
			"size: Queried matrix dimension must be 1 or 2. Called with i=%i",
			dim);
}
}  // namespace mrpt::math
