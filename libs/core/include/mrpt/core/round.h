/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/core/SSE_types.h>  // needed by SSE intrinsics used in some inline functions below.
#include <mrpt/core/cpu.h>
#include <cmath>  // pow(), lrint()

namespace mrpt
{
/** \addtogroup mrpt_round Round functions (in #include <mrpt/core/round.h>)
 *  \ingroup mrpt_core_grp
 * @{ */

/** Returns the closer integer (int) to x */
template <typename T>
inline int round(const T value)
{
#if MRPT_HAS_SSE2
	if (mrpt::cpu::supports(mrpt::cpu::feature::SSE2))
	{
		__m128d t = _mm_set_sd(value);
		return _mm_cvtsd_si32(t);
	}
	else
#endif
		return static_cast<int>(lrint(value));
}

/** Returns the closer integer (long) to x */
template <typename T>
inline long round_long(const T value)
{
#if MRPT_HAS_SSE2 && MRPT_WORD_SIZE == 64
	if (mrpt::cpu::supports(mrpt::cpu::feature::SSE2))
	{
		__m128d t = _mm_set_sd(value);
		return _mm_cvtsd_si64(t);
	}
	else
#endif
		return lrint(value);
}

/** Round a decimal number up to the given 10'th power (eg, to 1000,100,10, and
 * also fractions)
 *  power10 means round up to: 1 -> 10, 2 -> 100, 3 -> 1000, ...  -1 -> 0.1, -2
 * -> 0.01, ...
 */
template <class T>
T round_10power(T val, int power10)
{
	long double F = ::pow((long double)10.0, -(long double)power10);
	long int t = round_long(val * F);
	return T(t / F);
}

/** @} */
}  // namespace mrpt
