/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <algorithm>  // max()

namespace mrpt
{
/** Efficient and portable evaluation of the absolute difference of two
 * unsigned integer values; but also works for signed and floating point types
 * \ingroup mrpt_bits_math
 */
template <typename T>
inline T abs_diff(const T a, const T b)
{
	return std::max(a, b) - std::min(a, b);
}

}  // namespace mrpt