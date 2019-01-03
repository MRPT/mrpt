/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "img-precomp.h"  // Precompiled headers

// ---------------------------------------------------------------------------
//   This file contains the SSE3/SSSE3 optimized functions for
//   mrpt::img::CImage
//    See the sources and the doxygen documentation page "sse_optimizations" for
//    more details.
// ---------------------------------------------------------------------------
#if MRPT_HAS_SSE3

#include <mrpt/img/CImage.h>
#include <mrpt/core/SSE_types.h>
#include <mrpt/core/SSE_macros.h>
#include "CImage_SSEx.h"

/** \addtogroup sse_optimizations
 *  SSE optimized functions
 *  @{
 */

/** Subsample each 2x2 pixel block into 1x1 pixel, taking the first pixel &
 * ignoring the other 3
 *  - <b>Input format:</b> uint8_t, 3 channels (RGB or BGR)
 *  - <b>Output format:</b> uint8_t, 3 channels (RGB or BGR)
 *  - <b>Preconditions:</b> in & out aligned to 16bytes, step = k*16
 *  - <b>Notes:</b>
 *  - <b>Requires:</b> SSSE3
 *  - <b>Invoked from:</b> mrpt::img::CImage::scaleHalf()
 */
void image_SSSE3_scale_half_3c8u(
	const uint8_t* in, uint8_t* out, int w, int h, size_t step_in,
	size_t step_out)
{
	// clang-format off
#if defined(_MSC_VER)
#pragma warning( disable : 4309 ) // Yes, we know 0x80 is a "negative char"
#endif
	const __m128i m0 = _mm_set_epi8(0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x0E, 0x0D, 0x0C, 0x08, 0x07, 0x06, 0x02, 0x01, 0x00);
	const __m128i m1 = _mm_set_epi8(0x0E, 0x0A, 0x09, 0x08, 0x04, 0x03, 0x02, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80);
	const __m128i m2 = _mm_set_epi8(0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x0C, 0x0B, 0x0A, 0x06, 0x05, 0x04, 0x00, 0x80);
	const __m128i m3 = _mm_set_epi8(0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x0F);

	// clang-format on
#if defined(_MSC_VER)
#pragma warning(default : 4309)
#endif

	const int sw = w >> 4;  // This are the number of 3*16 blocks in each row
	const int sh = h >> 1;

	for (int i = 0; i < sh; i++)
	{
		const __m128i* inp = reinterpret_cast<const __m128i*>(in);
		uint8_t* outp = out;

		for (int j = 0; j < sw; j++)
		{
			// 16-byte blocks #0,#1,#2:
			__m128i d0 = _mm_load_si128(inp++);
			__m128i d1 = _mm_load_si128(inp++);

			// First 16 bytes:
			__m128i shuf0 = _mm_shuffle_epi8(d0, m0);
			__m128i shuf1 = _mm_shuffle_epi8(d1, m1);

			__m128i res0 = _mm_or_si128(shuf0, shuf1);

			_mm_storeu_si128((__m128i*)outp, res0);  // aligned output
			outp += 16;

			// Last 8 bytes:
			__m128i d2 = _mm_load_si128(inp++);

			_mm_storel_epi64(  // Write lower 8 bytes only
				(__m128i*)outp,
				_mm_or_si128(
					_mm_shuffle_epi8(d2, m2), _mm_shuffle_epi8(d1, m3)));
			outp += 8;
		}
		in += 2 * step_in;  // Skip one row
		out += step_out;
	}
}

// This is the actual function behind both: image_SSSE3_rgb_to_gray_8u() and
// image_SSSE3_bgr_to_gray_8u():
template <bool IS_RGB>
void private_image_SSSE3_rgb_or_bgr_to_gray_8u(
	const uint8_t* in, uint8_t* out, int w, int h, size_t step_in,
	size_t step_out)
{
	// clang-format off
#if defined(_MSC_VER)
#pragma warning( disable : 4309 ) // Yes, we know 0x80 is a "negative char"
#endif

	// Masks:                            0       1    2    3     4      5     6    7     8      9     A     B      C    D    E     F
	// reds[0-7] from D0
	const __m128i mask0 = _mm_setr_epi8(0x80, 0x00, 0x80, 0x03, 0x80, 0x06, 0x80, 0x09, 0x80, 0x0C, 0x80, 0x0F, 0x80, 0x80, 0x80, 0x80);
	// reds[0-7] from D1
	const __m128i mask1 = _mm_setr_epi8(0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x02, 0x80, 0x05);
	// greens[0-7] from D0
	const __m128i mask2 = _mm_setr_epi8(0x80, 0x01, 0x80, 0x04, 0x80, 0x07, 0x80, 0x0A, 0x80, 0x0D, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80);
	// greens[0-7] from D1
	const __m128i mask3 = _mm_setr_epi8(0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x80, 0x03, 0x80, 0x06);
	// blues[0-7] from D0
	const __m128i mask4 = _mm_setr_epi8(0x80, 0x02, 0x80, 0x05, 0x80, 0x08, 0x80, 0x0B, 0x80, 0x0E, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80);
	// blues[0-7] from D1
	const __m128i mask5 = _mm_setr_epi8(0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x01, 0x80, 0x04, 0x80, 0x07);
	// reds[8-15] from D1
	const __m128i mask6 = _mm_setr_epi8(0x80, 0x08, 0x80, 0x0B, 0x80, 0x0E, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80);
	// reds[8-15] from D2
	const __m128i mask7 = _mm_setr_epi8(0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x01, 0x80, 0x04, 0x80, 0x07, 0x80, 0x0A, 0x80, 0x0D);
	// greens[8-15] from D1
	const __m128i mask8 = _mm_setr_epi8(0x80, 0x09, 0x80, 0x0C, 0x80, 0x0F, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80);
	// greens[8-15] from D2
	const __m128i mask9 = _mm_setr_epi8(0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x02, 0x80, 0x05, 0x80, 0x08, 0x80, 0x0B, 0x80, 0x0E);
	// blues[8-15] from D1
	const __m128i mask10 = _mm_setr_epi8(0x80, 0x0A, 0x80, 0x0D, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80);
	// blues[8-15] from D2
	const __m128i mask11 = _mm_setr_epi8(0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x80, 0x03, 0x80, 0x06, 0x80, 0x09, 0x80, 0x0C, 0x80, 0x0F);
	// Conversion factors for RGB->Y
	const __m128i VAL_R = _mm_setr_epi8(0x00, 0x1D, 0x00, 0x1D, 0x00, 0x1D, 0x00, 0x1D, 0x00, 0x1D, 0x00, 0x1D, 0x00, 0x1D, 0x00, 0x1D);
	const __m128i VAL_G = _mm_setr_epi8(0x00, 0x96, 0x00, 0x96, 0x00, 0x96, 0x00, 0x96, 0x00, 0x96, 0x00, 0x96, 0x00, 0x96, 0x00, 0x96);
	const __m128i VAL_B = _mm_setr_epi8(0x00, 0x4D, 0x00, 0x4D, 0x00, 0x4D, 0x00, 0x4D, 0x00, 0x4D, 0x00, 0x4D, 0x00, 0x4D, 0x00, 0x4D);
	// mask:
	const __m128i mask_low = _mm_setr_epi8(0x01, 0x03, 0x05, 0x07, 0x09, 0x0B, 0x0D, 0x0F, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80);

	// clang-format on
#if defined(_MSC_VER)
#pragma warning(default : 4309)
#endif

	const __m128i m0 = IS_RGB ? mask4 : mask0;
	const __m128i m1 = IS_RGB ? mask5 : mask1;
	const __m128i m2 = mask2;
	const __m128i m3 = mask3;
	const __m128i m4 = IS_RGB ? mask0 : mask4;
	const __m128i m5 = IS_RGB ? mask1 : mask5;
	const __m128i m6 = IS_RGB ? mask10 : mask6;
	const __m128i m7 = IS_RGB ? mask11 : mask7;
	const __m128i m8 = mask8;
	const __m128i m9 = mask9;
	const __m128i m10 = IS_RGB ? mask6 : mask10;
	const __m128i m11 = IS_RGB ? mask7 : mask11;

	const int sw = w >> 4;  // This are the number of 3*16 blocks in each row
	const int sh = h;

	for (int i = 0; i < sh; i++)
	{
		const __m128i* inp = reinterpret_cast<const __m128i*>(in);
		uint8_t* outp = out;

		for (int j = 0; j < sw; j++)
		{
			// We process RGB data in blocks of 3 x 16byte blocks:
			const __m128i d0 = _mm_load_si128(inp++);
			const __m128i d1 = _mm_load_si128(inp++);
			const __m128i d2 = _mm_load_si128(inp++);

			// First 8 bytes of gray levels:
			{
				const __m128i BLUES_0_7 = _mm_or_si128(
					_mm_shuffle_epi8(d0, m0), _mm_shuffle_epi8(d1, m1));
				const __m128i GREENS_0_7 = _mm_or_si128(
					_mm_shuffle_epi8(d0, m2), _mm_shuffle_epi8(d1, m3));
				const __m128i REDS_0_7 = _mm_or_si128(
					_mm_shuffle_epi8(d0, m4), _mm_shuffle_epi8(d1, m5));

				// _mm_mulhi_epu16(): Multiplies the 8 unsigned 16-bit integers
				// from a by the 8 unsigned 16-bit integers from b.
				// r0 := (a0 * b0)[31:16]
				// r1 := (a1 * b1)[31:16]
				//...
				// r7 := (a7 * b7)[31:16]
				//
				const __m128i GRAYS_0_7 = _mm_adds_epu16(
					_mm_mulhi_epu16(REDS_0_7, VAL_R),
					_mm_adds_epu16(
						_mm_mulhi_epu16(GREENS_0_7, VAL_G),
						_mm_mulhi_epu16(BLUES_0_7, VAL_B)));

				_mm_storel_epi64(
					reinterpret_cast<__m128i*>(outp),
					_mm_shuffle_epi8(GRAYS_0_7, mask_low));
				outp += 8;
			}

			// Second 8 bytes of gray levels:
			{
				const __m128i BLUES_8_15 = _mm_or_si128(
					_mm_shuffle_epi8(d1, m6), _mm_shuffle_epi8(d2, m7));
				const __m128i GREENS_8_15 = _mm_or_si128(
					_mm_shuffle_epi8(d1, m8), _mm_shuffle_epi8(d2, m9));
				const __m128i REDS_8_15 = _mm_or_si128(
					_mm_shuffle_epi8(d1, m10), _mm_shuffle_epi8(d2, m11));

				const __m128i GRAYS_8_15 = _mm_adds_epu16(
					_mm_mulhi_epu16(REDS_8_15, VAL_R),
					_mm_adds_epu16(
						_mm_mulhi_epu16(GREENS_8_15, VAL_G),
						_mm_mulhi_epu16(BLUES_8_15, VAL_B)));

				_mm_storel_epi64(
					reinterpret_cast<__m128i*>(outp),
					_mm_shuffle_epi8(GRAYS_8_15, mask_low));
				outp += 8;
			}
		}
		in += step_in;
		out += step_out;
	}

}  // end private_image_SSSE3_rgb_or_bgr_to_gray_8u()

/** Convert a RGB image (3cu8) into a GRAYSCALE (1c8u) image, using
 * Y=77*R+150*G+29*B
 *  - <b>Input format:</b> uint8_t, 3 channels (BGR order)
 *  - <b>Output format:</b> uint8_t, 1 channel
 *  - <b>Preconditions:</b> in & out aligned to 16bytes, step = k*16
 *  - <b>Notes:</b>
 *  - <b>Requires:</b> SSSE3
 *  - <b>Invoked from:</b> mrpt::img::CImage::grayscale(),
 * mrpt::img::CImage::grayscaleInPlace()
 */
void image_SSSE3_bgr_to_gray_8u(
	const uint8_t* in, uint8_t* out, int w, int h, size_t step_in,
	size_t step_out)
{
	private_image_SSSE3_rgb_or_bgr_to_gray_8u<false>(
		in, out, w, h, step_in, step_out);
}

/** Convert a RGB image (3cu8) into a GRAYSCALE (1c8u) image, using
 * Y=77*R+150*G+29*B
 *  - <b>Input format:</b> uint8_t, 3 channels (RGB order)
 *  - <b>Output format:</b> uint8_t, 1 channel
 *  - <b>Preconditions:</b> in & out aligned to 16bytes, step = k*16
 *  - <b>Notes:</b>
 *  - <b>Requires:</b> SSSE3
 *  - <b>Invoked from:</b> mrpt::img::CImage::grayscale(),
 * mrpt::img::CImage::grayscaleInPlace()
 */
void image_SSSE3_rgb_to_gray_8u(
	const uint8_t* in, uint8_t* out, int w, int h, size_t step_in,
	size_t step_out)
{
	private_image_SSSE3_rgb_or_bgr_to_gray_8u<true>(
		in, out, w, h, step_in, step_out);
}

/**  @} */

#endif  // end of MRPT_HAS_SSE3
