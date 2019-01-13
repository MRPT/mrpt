/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "img-precomp.h"  // Precompiled headers

#if MRPT_HAS_SSE2
// ---------------------------------------------------------------------------
//   This file contains the SSE2 optimized functions for mrpt::img::CImage
//    See the sources and the doxygen documentation page "sse_optimizations" for
//    more details.
//
//  Some functions here are derived from sources in libcvd, released
//   under BSD. https://www.edwardrosten.com/cvd/
//
// ---------------------------------------------------------------------------

#include <mrpt/img/CImage.h>
#include <mrpt/system/memory.h>
#include <mrpt/core/SSE_types.h>
#include <mrpt/core/SSE_macros.h>
#include "CImage_SSEx.h"

/** \addtogroup sse_optimizations
 *  SSE optimized functions
 *  @{
 */

/** Subsample each 2x2 pixel block into 1x1 pixel, taking the first pixel &
 * ignoring the other 3
 *  - <b>Input format:</b> uint8_t, 1 channel
 *  - <b>Output format:</b> uint8_t, 1 channel
 *  - <b>Preconditions:</b> in & out aligned to 16bytes, step = k*16
 *  - <b>Notes:</b>
 *  - <b>Requires:</b> SSE2
 *  - <b>Invoked from:</b> mrpt::img::CImage::scaleHalf()
 */
void image_SSE2_scale_half_1c8u(
	const uint8_t* in, uint8_t* out, int w, int h, size_t step_in,
	size_t step_out)
{
	MRPT_TODO("Fix: allow unaligned");

	ASSERT_(mrpt::system::is_aligned<16>(in));
	ASSERT_(mrpt::system::is_aligned<16>(out));
	ASSERTMSG_((step_in & 0x0f) == 0, "step of input image must be 16*k");
	ASSERTMSG_((step_out & 0x0f) == 0, "step of output image must be 16*k");

	// clang-format off
#if defined(_MSC_VER)
#pragma warning( disable : 4309 ) // Yes, we know 0x80 is a "negative char"
#endif
	const __m128i m = _mm_set_epi8(0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff);

	// clang-format on
#if defined(_MSC_VER)
#pragma warning(default : 4309)
#endif

	int sw = w >> 4;
	int sh = h >> 1;

	for (int i = 0; i < sh; i++)
	{
		for (int j = 0; j < sw; j++)
		{
			auto inp = reinterpret_cast<const __m128i*>(in);
			uint8_t* outp = out;

			const __m128i x = _mm_and_si128(_mm_load_si128(inp++), m);
			auto o = reinterpret_cast<__m128i*>(outp);
			_mm_storel_epi64(o, _mm_packus_epi16(x, x));
			outp += 8;
		}
		in += 2 * step_in;  // Skip one row
		out += step_out;
	}
}

/** Average each 2x2 pixels into 1x1 pixel (arithmetic average)
 *  - <b>Input format:</b> uint8_t, 1 channel
 *  - <b>Output format:</b> uint8_t, 1 channel
 *  - <b>Preconditions:</b> in & out aligned to 16bytes, step = k*16
 *  - <b>Notes:</b>
 *  - <b>Requires:</b> SSE2
 *  - <b>Invoked from:</b> mrpt::img::CImage::scaleHalfSmooth()
 */
void image_SSE2_scale_half_smooth_1c8u(
	const uint8_t* in, uint8_t* out, int w, int h, size_t step_in,
	size_t step_out)
{
	MRPT_TODO("Fix: allow unaligned");

	ASSERT_(mrpt::system::is_aligned<16>(in));
	ASSERT_(mrpt::system::is_aligned<16>(out));
	ASSERTMSG_((step_in & 0x0f) == 0, "step of input image must be 16*k");
	ASSERTMSG_((step_out & 0x0f) == 0, "step of output image must be 16*k");

	// clang-format off
#if defined(_MSC_VER)
#pragma warning( disable : 4309 ) // Yes, we know 0x80 is a "negative char"
#endif
	const __m128i m = _mm_set_epi8(0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff);

	// clang-format on
#if defined(_MSC_VER)
#pragma warning(default : 4309)
#endif

	int sw = w >> 4;
	int sh = h >> 1;

	for (int i = 0; i < sh; i++)
	{
		auto inp = reinterpret_cast<const __m128i*>(in);
		auto nextRow = reinterpret_cast<const __m128i*>(in + step_in);
		uint8_t* outp = out;

		for (int j = 0; j < sw; j++)
		{
			__m128i here = _mm_load_si128(inp++);
			__m128i next = _mm_load_si128(nextRow++);
			here = _mm_avg_epu8(here, next);
			next = _mm_and_si128(_mm_srli_si128(here, 1), m);
			here = _mm_and_si128(here, m);
			here = _mm_avg_epu16(here, next);
			_mm_storel_epi64(
				reinterpret_cast<__m128i*>(outp), _mm_packus_epi16(here, here));
			outp += 8;
		}
		in += 2 * step_in;  // Skip one row
		out += step_out;
	}
}

// TODO:
// Sum of absolute differences: Use  _mm_sad_epu8

/**  @} */

#endif  // end if MRPT_HAS_SSE2
