/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/utils/SSE_types.h>  // needed by SSE intrinsics used in some inline functions below.

namespace mrpt
{
	namespace utils
	{
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

	} // End of namespace
} // end of namespace
