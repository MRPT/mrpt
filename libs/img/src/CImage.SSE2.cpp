/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "img-precomp.h"  // Precompiled headers
//
#include <mrpt/config.h>

#if MRPT_ARCH_INTEL_COMPATIBLE
// ---------------------------------------------------------------------------
//   This file contains the SSE2 optimized functions for mrpt::img::CImage
//    See the sources and the doxygen documentation page "sse_optimizations" for
//    more details.
//
//  Some functions here are derived from sources in libcvd, released
//   under BSD. https://www.edwardrosten.com/cvd/
//
// ---------------------------------------------------------------------------

#include <mrpt/core/SSE_macros.h>
#include <mrpt/core/SSE_types.h>
#include <mrpt/img/CImage.h>
#include <mrpt/system/memory.h>

#include "CImage.SSEx.h"

/** \addtogroup sse_optimizations
 *  SSE optimized functions
 *  @{
 */

template <bool MemIsAligned>
void impl_image_SSE2_scale_half_1c8u(
	const uint8_t* in, uint8_t* out, int w, int h, size_t step_in,
	size_t step_out)
{
	SSE_DISABLE_WARNINGS
	// clang-format off

	const __m128i m = _mm_set_epi8(0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff);

	// clang-format on
	SSE_RESTORE_SIGN_WARNINGS

	const int sw = w / 16;
	const int sh = h / 2;
	const int rest_w = w - (16 * w);

	for (int i = 0; i < sh; i++)
	{
		auto inp = reinterpret_cast<const __m128i*>(in);
		uint8_t* outp = out;
		for (int j = 0; j < sw; j++)
		{
			const __m128i x =
				_mm_and_si128(mm_load_si128<MemIsAligned>(inp++), m);
			auto o = reinterpret_cast<__m128i*>(outp);
			_mm_storel_epi64(o, _mm_packus_epi16(x, x));
			outp += 8;
		}
		// Extra pixels? (w mod 16 != 0)
		if (rest_w != 0)
		{
			const uint8_t* in_rest = in + 16 * sw;
			for (int p = 0; p < rest_w / 2; p++)
			{
				*outp++ = in_rest[0];
				in_rest += 2;
			}
		}

		in += 2 * step_in;	// Skip one row
		out += step_out;
	}
}

template <bool MemIsAligned>
void impl_image_SSE2_scale_half_smooth_1c8u(
	const uint8_t* in, uint8_t* out, int w, int h, size_t step_in,
	size_t step_out)
{
	SSE_DISABLE_WARNINGS
	// clang-format off

	const __m128i m = _mm_set_epi8(0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff);

	// clang-format on
	SSE_RESTORE_SIGN_WARNINGS

	const int sw = w / 16;
	const int sh = h / 2;
	const int rest_w = w - (16 * w);

	for (int i = 0; i < sh; i++)
	{
		auto inp = reinterpret_cast<const __m128i*>(in);
		auto nextRow = reinterpret_cast<const __m128i*>(in + step_in);
		uint8_t* outp = out;

		for (int j = 0; j < sw; j++)
		{
			__m128i here = mm_load_si128<MemIsAligned>(inp++);
			__m128i next = mm_load_si128<MemIsAligned>(nextRow++);
			here = _mm_avg_epu8(here, next);
			next = _mm_and_si128(_mm_srli_si128(here, 1), m);
			here = _mm_and_si128(here, m);
			here = _mm_avg_epu16(here, next);
			_mm_storel_epi64(
				reinterpret_cast<__m128i*>(outp), _mm_packus_epi16(here, here));
			outp += 8;
		}

		// Extra pixels? (w mod 16 != 0)
		if (rest_w != 0)
		{
			const uint8_t* ir = in + 16 * sw;
			const uint8_t* irr = in + step_in;
			for (int p = 0; p < rest_w / 2; p++)
			{
				*outp++ = (ir[0] + ir[1] + irr[0] + irr[1]) / 4;
				ir += 2;
				irr += 2;
			}
		}

		in += 2 * step_in;	// Skip one row
		out += step_out;
	}
}

/** Subsample each 2x2 pixel block into 1x1 pixel, taking the first pixel &
 * ignoring the other 3
 *  - <b>Input format:</b> uint8_t, 1 channel
 *  - <b>Output format:</b> uint8_t, 1 channel
 *  - <b>Preconditions:</b> in & out aligned to 16bytes (faster) or not, step =
 * k*16 (faster) or not
 *  - <b>Notes:</b>
 *  - <b>Requires:</b> SSE2
 *  - <b>Invoked from:</b> mrpt::img::CImage::scaleHalf()
 */
void image_SSE2_scale_half_1c8u(
	const uint8_t* in, uint8_t* out, int w, int h, size_t step_in,
	size_t step_out)
{
	if (mrpt::system::is_aligned<16>(in) && mrpt::system::is_aligned<16>(out) &&
		is_multiple<16>(step_in) && is_multiple<16>(step_out))
	{
		impl_image_SSE2_scale_half_1c8u<true>(in, out, w, h, step_in, step_out);
	}
	else
	{
		impl_image_SSE2_scale_half_1c8u<false>(
			in, out, w, h, step_in, step_out);
	}
}

/** Average each 2x2 pixels into 1x1 pixel (arithmetic average)
 *  - <b>Input format:</b> uint8_t, 1 channel
 *  - <b>Output format:</b> uint8_t, 1 channel
 *  - <b>Preconditions:</b> in & out aligned to 16bytes (faster) or not, step =
 * k*16 (faster) or not
 *  - <b>Notes:</b>
 *  - <b>Requires:</b> SSE2
 *  - <b>Invoked from:</b> mrpt::img::CImage::scaleHalfSmooth()
 */
void image_SSE2_scale_half_smooth_1c8u(
	const uint8_t* in, uint8_t* out, int w, int h, size_t step_in,
	size_t step_out)
{
	if (mrpt::system::is_aligned<16>(in) && mrpt::system::is_aligned<16>(out) &&
		is_multiple<16>(step_in) && is_multiple<16>(step_out))
	{
		impl_image_SSE2_scale_half_smooth_1c8u<true>(
			in, out, w, h, step_in, step_out);
	}
	else
	{
		impl_image_SSE2_scale_half_smooth_1c8u<false>(
			in, out, w, h, step_in, step_out);
	}
}

// TODO:
// Sum of absolute differences: Use  _mm_sad_epu8

/**  @} */

#endif	// end if MRPT_ARCH_INTEL_COMPATIBLE
