/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
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

#include <mrpt/base.h>  // Precompiled headers

// ---------------------------------------------------------------------------
//   This file contains the SSE3/SSSE3 optimized functions for mrpt::utils::CImage
//    See the sources and the doxygen documentation page "sse_optimizations" for more details.
// ---------------------------------------------------------------------------
#if MRPT_HAS_SSE3

#include <mrpt/utils/CImage.h>
#include "CImage_SSEx.h"


/** \addtogroup sse_optimizations
 *   SSE optimized functions
 *  @{
 */

/** Subsample each 2x2 pixel block into 1x1 pixel, taking the first pixel & ignoring the other 3.
  *  - <b>Input format:</b> uint8_t, 3 channels (RGB or BGR)
  *  - <b>Output format:</b> uint8_t, 3 channels (RGB or BGR)
  *  - <b>Preconditions:</b> in & out aligned to 16bytes, w = k*16 (w=width in pixels)
  *  - <b>Notes:</b> 
  *  - <b>Invoked from:</b> mrpt::utils::CImage::scaleHalf()
  */
void image_SSSE3_scale_half_3c8u(const uint8_t* in, uint8_t* out, int w, int h)
{
	EIGEN_ALIGN16 const unsigned long long mask0[2] = { 0x0D0C080706020100ull, 0x808080808080800Eull }; // Long words are in inverse order due to little endianness
	EIGEN_ALIGN16 const unsigned long long mask1[2] = { 0x8080808080808080ull, 0x0E0A090804030280ull };
	EIGEN_ALIGN16 const unsigned long long mask2[2] = { 0x0C0B0A0605040080ull, 0x8080808080808080ull };
	EIGEN_ALIGN16 const unsigned long long mask3[2] = { 0x808080808080800Full, 0x8080808080808080ull };

	const __m128i m0 = _mm_load_si128((const __m128i*)mask0);
	const __m128i m1 = _mm_load_si128((const __m128i*)mask1);
	const __m128i m2 = _mm_load_si128((const __m128i*)mask2);
	const __m128i m3 = _mm_load_si128((const __m128i*)mask3);

	const int sw = w >> 4;  // This are the number of 3*16 blocks in each row
	const int sh = h >> 1;

	for (int i=0; i<sh; i++)
	{
		for (int j=0; j<sw; j++)
		{
			// 16-byte blocks #0,#1,#2:
			__m128i d0 = _mm_load_si128((const __m128i*)in); in += 16;
			__m128i d1 = _mm_load_si128((const __m128i*)in); in += 16;

			// First 16 bytes:
			__m128i shuf0 = _mm_shuffle_epi8(d0,m0);
			__m128i shuf1 = _mm_shuffle_epi8(d1,m1);

			__m128i res0 = _mm_or_si128(shuf0,shuf1);

			if ((j&0x1)!=0)
			     _mm_storeu_si128((__m128i*)out,res0);  // unaligned output
			else _mm_store_si128 ((__m128i*)out,res0);  // aligned output
			out += 16;

			// Last 8 bytes:
			__m128i d2 = _mm_load_si128((const __m128i*)in); in += 16;

			_mm_storel_epi64(	// Write lower 8 bytes only
				(__m128i*)out,
				_mm_or_si128(_mm_shuffle_epi8(d2,m2),_mm_shuffle_epi8(d1,m3))
				);
			out += 8;
		}
		in += 3*w;
	}
}

/**  @} */

#endif // end of MRPT_HAS_SSE3
