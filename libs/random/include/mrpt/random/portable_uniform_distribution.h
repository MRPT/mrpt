/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <cstdint>

namespace mrpt::random
{
/** A compiler and platform-independent uniform distribution generator.
 *  \tparam URGB Can be a C++11 std random generator, or
 *          mrpt::random::Generator_MT19937
 *\ingroup mrpt_random_grp
 */
template <class URBG>
uint64_t portable_uniform_distribution(URBG&& g, uint64_t min, uint64_t max)
{
	if (min == max) return min;

	const uint64_t sample = (static_cast<uint64_t>(g()) << 32) | g();
	return min + (sample % (max - min));
}

}  // namespace mrpt::random
