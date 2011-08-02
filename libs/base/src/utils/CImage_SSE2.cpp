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

#include <mrpt/base.h>  // Precompiled headers

#if MRPT_HAS_SSE2
// ---------------------------------------------------------------------------
//   This file contains the SSE2 optimized functions for mrpt::utils::CImage
//    See the sources and the doxygen documentation page "sse_optimizations" for more details.
//
//  Some functions here are derived from sources in libcvd, released
//   under LGPL. See http://mi.eng.cam.ac.uk/~er258/cvd/
//
// ---------------------------------------------------------------------------

#include <mrpt/utils/CImage.h>
#include <mrpt/utils/SSE_types.h>
#include <mrpt/utils/SSE_macros.h>
#include "CImage_SSEx.h"

/** \addtogroup sse_optimizations
 *  SSE optimized functions
 *  @{
 */

/** Subsample each 2x2 pixel block into 1x1 pixel, taking the first pixel & ignoring the other 3
  *  - <b>Input format:</b> uint8_t, 1 channel
  *  - <b>Output format:</b> uint8_t, 1 channel
  *  - <b>Preconditions:</b> in & out aligned to 16bytes, w = k*16 (w=width in pixels), widthStep=w*1
  *  - <b>Notes:</b>
  *  - <b>Requires:</b> SSE2
  *  - <b>Invoked from:</b> mrpt::utils::CImage::scaleHalf()
  */
void image_SSE2_scale_half_1c8u(const uint8_t* in, uint8_t* out, int w, int h)
{
	EIGEN_ALIGN16 const unsigned long long mask[2] = {0x00FF00FF00FF00FFull, 0x00FF00FF00FF00FFull};
	const __m128i m = _mm_load_si128((const __m128i*)mask);

	int sw = w >> 4;
	int sh = h >> 1;

	for (int i=0; i<sh; i++)
	{
		for (int j=0; j<sw; j++)
		{
			const __m128i here_sampled = _mm_and_si128( _mm_load_si128((const __m128i*)in), m);
			_mm_storel_epi64((__m128i*)out, _mm_packus_epi16(here_sampled,here_sampled));
			in += 16;
			out += 8;
		}
		in += w;
	}
}


/** Average each 2x2 pixels into 1x1 pixel (arithmetic average)
  *  - <b>Input format:</b> uint8_t, 1 channel
  *  - <b>Output format:</b> uint8_t, 1 channel
  *  - <b>Preconditions:</b> in & out aligned to 16bytes, w = k*16 (w=width in pixels), widthStep=w*1
  *  - <b>Notes:</b>
  *  - <b>Requires:</b> SSE2
  *  - <b>Invoked from:</b> mrpt::utils::CImage::scaleHalfSmooth()
  */
void image_SSE2_scale_half_smooth_1c8u(const uint8_t* in, uint8_t* out, int w, int h)
{
	EIGEN_ALIGN16 const unsigned long long mask[2] = {0x00FF00FF00FF00FFull, 0x00FF00FF00FF00FFull};
	const uint8_t* nextRow = in + w;
	__m128i m = _mm_load_si128((const __m128i*)mask);
	int sw = w >> 4;
	int sh = h >> 1;

	for (int i=0; i<sh; i++)
	{
		for (int j=0; j<sw; j++)
		{
			__m128i here = _mm_load_si128((const __m128i*)in);
			__m128i next = _mm_load_si128((const __m128i*)nextRow);
			here = _mm_avg_epu8(here,next);
			next = _mm_and_si128(_mm_srli_si128(here,1), m);
			here = _mm_and_si128(here,m);
			here = _mm_avg_epu16(here, next);
			_mm_storel_epi64((__m128i*)out, _mm_packus_epi16(here,here));
			in += 16;
			nextRow += 16;
			out += 8;
		}

		in += w;
		nextRow += w;
	}
}



/** KLT score at a given point of a grayscale image.
  *  - <b>Requires:</b> SSE2
  *  - <b>Invoked from:</b> mrpt::utils::CImage::KLT_response()
  *
  *  This function is not manually optimized for SSE2 but templatized for different
  *   window sizes such as the compiler can optimize automatically for that size.
  *
  *  Only for the most common window sizes this templates are instantiated (W=[2-16] and W=32 ),
  *   falling back to
  *   a generic implementation otherwise. The next figure shows the performance (time for
  *   KLT_response() to compute the score for one single pixel) for different window sizes.
  *
  *  <img src="KLT_response_performance_SSE2.png" >
  *
  */
float KLT_response_optimized();

// TODO:
// Sum of absolute differences: Use  _mm_sad_epu8

/**  @} */

#endif // end if MRPT_HAS_SSE2
