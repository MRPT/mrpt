/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once


namespace mrpt
{
namespace math
{
// This inline function is used everywhere, so just move it here even it's not a
// forward declaration!
/*! Returns the size of the matrix in the i'th dimension: 1=rows, 2=columns
 * (MATLAB-compatible function)
  *  \note Template argument MATRIXLIKE can be: mrpt::math::CMatrixTemplate,
 * mrpt::math::CMatrixTemplateNumeric, mrpt::math::CMatrixFixedNumeric
  */
template <class MATRIXLIKE>
inline size_t size(const MATRIXLIKE& m, const int dim)
{
	if (dim == 1)
		return m.getRowCount();
	else if (dim == 2)
		return m.getColCount();
	else
		THROW_EXCEPTION_FMT(
			"size: Queried matrix dimension must be 1 or 2. Called with i=%i",
			dim);
}

}  // End of namespace
}  // end of namespace
