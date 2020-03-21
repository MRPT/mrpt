/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <array>

namespace mrpt::math
{
/** Auxiliary class used in CMatrixDynamic:size(), CMatrixDynamic::resize(),
 * CMatrixFixed::size(), CMatrixFixed::resize(), to mimic the
 * behavior of STL-containers.
 * \ingroup mrpt_math_grp
 */
struct matrix_size_t : public std::array<std::size_t, 2>
{
	constexpr matrix_size_t() : std::array<std::size_t, 2>{0, 0} {}
	constexpr matrix_size_t(const std::size_t rows, const std::size_t cols)
		: std::array<std::size_t, 2>{rows, cols}
	{
	}

	/** Cast to size_t as the overall number of matrix/vector elements */
	operator std::size_t() const { return at(0) * at(1); }
};

}  // namespace mrpt::math
