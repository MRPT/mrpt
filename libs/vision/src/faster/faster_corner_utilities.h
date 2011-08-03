/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
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
