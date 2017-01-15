/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/utils/SSE_types.h>  // needed by SSE intrinsics used in some inline functions below.
#define _USE_MATH_DEFINES // (For VS to define M_PI, etc. in cmath)
#include <cmath> // pow()

namespace mrpt
{
	namespace utils
	{
		/** \addtogroup mrpt_round Round functions (in #include <mrpt/utils/round.h>)
		  *  \ingroup mrpt_base_grp
		  * @{ */

		/** Returns the closer integer (int) to x */
		template <typename T>
		inline int round(const T value)
		{
		#if MRPT_HAS_SSE2
			__m128d t = _mm_set_sd( value );
			return _mm_cvtsd_si32(t);
		#elif (defined WIN32 || defined _WIN32) && !defined WIN64 && !defined _WIN64 && defined _MSC_VER
			int t;
			__asm
			{
				fld value;
				fistp t;
			}
			return t;
		#elif defined HAVE_LRINT || defined __GNUC__
			return static_cast<int>(lrint(value));
		#else
			return static_cast<int>(value + 0.5);
		#endif
		}

		/** Returns the closer integer (long) to x */
		template <typename T>
		inline long round_long(const T value)
		{
		#if MRPT_HAS_SSE2 && MRPT_WORD_SIZE==64
			__m128d t = _mm_set_sd( value );
			return _mm_cvtsd_si64(t);
		#elif (defined WIN32 || defined _WIN32) && !defined WIN64 && !defined _WIN64 && defined _MSC_VER
			long t;
			__asm
			{
				fld value;
				fistp t;
			}
			return t;
		#elif defined HAVE_LRINT || defined __GNUC__
			return lrint(value);
		#else
			return static_cast<long>(value + 0.5);
		#endif
		}

		/** Round a decimal number up to the given 10'th power (eg, to 1000,100,10, and also fractions)
		  *  power10 means round up to: 1 -> 10, 2 -> 100, 3 -> 1000, ...  -1 -> 0.1, -2 -> 0.01, ...
		  */
		template <class T>
		T round_10power(T val, int power10)
		{
			long double F = ::pow((long double)10.0,-(long double)power10);
			long int t = mrpt::utils::round_long( val * F );
			return T(t/F);
		}

		/** @} */
	} // End of namespace
} // end of namespace
