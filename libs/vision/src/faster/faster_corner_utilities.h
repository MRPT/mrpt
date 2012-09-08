/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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
