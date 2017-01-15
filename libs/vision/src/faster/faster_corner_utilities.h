/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

// ---------------------------------------------------------------------------
// LICENSING: This file is a slightly-modified version of part of libcvd, 
//             released under LGPL 2.1 by Edward Rosten
// ---------------------------------------------------------------------------

#ifndef MRPT_CVD_INTERNAL_INC_FAST_CORNER_UTILITIES_H
#define MRPT_CVD_INTERNAL_INC_FAST_CORNER_UTILITIES_H

#include <mrpt/utils/SSE_types.h>

namespace mrpt
{
  struct Less 
  { 
    template <class T1, class T2> static bool eval(const T1 a, const T2 b) 
    {
	return a < b; 
    }
    static int prep_t(int pixel_val, int barrier)
    {
      return pixel_val - barrier;
    }
  };
  struct Greater 
  {
    template <class T1, class T2> static bool eval(const T1 a, const T2 b) 
    {
      return a > b; 
    }
    static int prep_t(int pixel_val, int barrier)
    {
      return pixel_val + barrier;
    }
  };

#if MRPT_HAS_SSE2

	#define CHECK_BARRIER(lo, hi, other, flags)				\
		{									\
		__m128i diff = _mm_subs_epu8(lo, other);			\
		__m128i diff2 = _mm_subs_epu8(other, hi);			\
		__m128i z = _mm_setzero_si128();				\
		diff = _mm_cmpeq_epi8(diff, z);					\
		diff2 = _mm_cmpeq_epi8(diff2, z);				\
		flags = ~(_mm_movemask_epi8(diff) | (_mm_movemask_epi8(diff2) << 16)); \
		}
     
    template <bool Aligned> inline __m128i load_si128(const void* addr) { return _mm_loadu_si128((const __m128i*)addr); }
    template <> inline __m128i load_si128<true>(const void* addr) { return _mm_load_si128((const __m128i*)addr); }

#endif

}
#endif
