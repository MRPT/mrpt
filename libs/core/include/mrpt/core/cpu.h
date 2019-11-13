/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/core/common.h>
#include <string>

namespace mrpt::cpu
{
/** OS-portable set of CPU feature definitions, for usage in mrpt::cpu::supports
 * \ingroup mrpt_core_grp
 */
enum class feature : unsigned int
{
	MMX = 0,
	POPCNT,
	SSE,
	SSE2,
	SSE3,
	SSSE3,
	SSE4_1,
	SSE4_2,
	AVX,
	AVX2,
	// ---- end of list ----
	FEATURE_COUNT
};

/** Returns true if CPU (and OS) supports the given CPU feature
 * \ingroup mrpt_core_grp
 */
bool supports(feature f);

/** Returns a string with detected features: "MMX:1 SSE2:0 etc."
 * \ingroup mrpt_core_grp
 */
std::string features_as_string();

}  // namespace mrpt::cpu
