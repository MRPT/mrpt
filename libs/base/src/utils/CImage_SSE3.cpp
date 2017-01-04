/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

// ---------------------------------------------------------------------------
//   This file contains the SSE3/SSSE3 optimized functions for mrpt::utils::CImage
//    See the sources and the doxygen documentation page "sse_optimizations" for more details.
// ---------------------------------------------------------------------------
#if MRPT_HAS_SSE3

#include <mrpt/utils/CImage.h>
#include <mrpt/utils/SSE_types.h>
#include <mrpt/utils/SSE_macros.h>
#include "CImage_SSEx.h"


/** \addtogroup sse_optimizations
 *  SSE optimized functions
 *  @{
 */

/** Subsample each 2x2 pixel block into 1x1 pixel, taking the first pixel & ignoring the other 3
  *  - <b>Input format:</b> uint8_t, 3 channels (RGB or BGR)
  *  - <b>Output format:</b> uint8_t, 3 channels (RGB or BGR)
  *  - <b>Preconditions:</b> in & out aligned to 16bytes, w = k*16 (w=width in pixels), widthStep=w*3
  *  - <b>Notes:</b>
  *  - <b>Requires:</b> SSSE3
  *  - <b>Invoked from:</b> mrpt::utils::CImage::scaleHalf()
  */
void image_SSSE3_scale_half_3c8u(const uint8_t* in, uint8_t* out, int w, int h)
{
	MRPT_ALIGN16 const unsigned long long mask0[2] = { 0x0D0C080706020100ull, 0x808080808080800Eull }; // Long words are in inverse order due to little endianness
	MRPT_ALIGN16 const unsigned long long mask1[2] = { 0x8080808080808080ull, 0x0E0A090804030280ull };
	MRPT_ALIGN16 const unsigned long long mask2[2] = { 0x0C0B0A0605040080ull, 0x8080808080808080ull };
	MRPT_ALIGN16 const unsigned long long mask3[2] = { 0x808080808080800Full, 0x8080808080808080ull };

	const __m128i m0 = _mm_load_si128((const __m128i*)mask0);
	const __m128i m1 = _mm_load_si128((const __m128i*)mask1);
	const __m128i m2 = _mm_load_si128((const __m128i*)mask2);
	const __m128i m3 = _mm_load_si128((const __m128i*)mask3);

	const int sw = w >> 4;  // This are the number of 3*16 blocks in each row
	const int sh = h >> 1;

	int odd_row = 0; 

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

			if ((odd_row&0x1)!=0)
			     _mm_storeu_si128((__m128i*)out,res0);  // unaligned output
			else _mm_store_si128 ((__m128i*)out,res0);  // aligned output
			out += 16;

			// Last 8 bytes:
			__m128i d2 = _mm_load_si128((const __m128i*)in); in += 16;

			_mm_storel_epi64(	// Write lower 8 bytes only
				(__m128i*)out,
				_mm_or_si128(_mm_shuffle_epi8(d2,m2),_mm_shuffle_epi8(d1,m3))
				);
			odd_row++;
			out += 8;
		}
		in += 3*w;
	}
}


// This is the actual function behind both: image_SSSE3_rgb_to_gray_8u() and image_SSSE3_bgr_to_gray_8u():
template <bool IS_RGB>
void private_image_SSSE3_rgb_or_bgr_to_gray_8u(const uint8_t* in, uint8_t* out, int w, int h)
{
	// Masks:                 0  1   2  3   4  5   6  7   8 9    A  B   C  D  E  F
	BUILD_128BIT_CONST(mask0, 80,00, 80,03, 80,06, 80,09, 80,0C, 80,0F, 80,80, 80,80) // reds[0-7] from D0
	BUILD_128BIT_CONST(mask1, 80,80, 80,80, 80,80, 80,80, 80,80, 80,80, 80,02, 80,05) // reds[0-7] from D1

	BUILD_128BIT_CONST(mask2, 80,01, 80,04, 80,07, 80,0A, 80,0D, 80,80, 80,80, 80,80) // greens[0-7] from D0
	BUILD_128BIT_CONST(mask3, 80,80, 80,80, 80,80, 80,80, 80,80, 80,00, 80,03, 80,06) // greens[0-7] from D1

	BUILD_128BIT_CONST(mask4, 80,02, 80,05, 80,08, 80,0B, 80,0E, 80,80, 80,80, 80,80) // blues[0-7] from D0
	BUILD_128BIT_CONST(mask5, 80,80, 80,80, 80,80, 80,80, 80,80, 80,01, 80,04, 80,07) // blues[0-7] from D1


	BUILD_128BIT_CONST(mask6, 80,08, 80,0B, 80,0E, 80,80, 80,80, 80,80, 80,80, 80,80) // reds[8-15] from D1
	BUILD_128BIT_CONST(mask7, 80,80, 80,80, 80,80, 80,01, 80,04, 80,07, 80,0A, 80,0D) // reds[8-15] from D2

	BUILD_128BIT_CONST(mask8, 80,09, 80,0C, 80,0F, 80,80, 80,80, 80,80, 80,80, 80,80) // greens[8-15] from D1
	BUILD_128BIT_CONST(mask9, 80,80, 80,80, 80,80, 80,02, 80,05, 80,08, 80,0B, 80,0E) // greens[8-15] from D2

	BUILD_128BIT_CONST(mask10,80,0A, 80,0D, 80,80, 80,80, 80,80, 80,80, 80,80, 80,80) // blues[8-15] from D1
	BUILD_128BIT_CONST(mask11,80,80, 80,80, 80,00, 80,03, 80,06, 80,09, 80,0C, 80,0F) // blues[8-15] from D2


	BUILD_128BIT_CONST(mask_to_low, 01,03,05,07,09,0B,0D,0F, 80,80,80,80,80,80,80,80)


	// Conversion factors for RGB->Y
	BUILD_128BIT_CONST(val_red   , 00,1D, 00,1D, 00,1D, 00,1D, 00,1D, 00,1D, 00,1D, 00,1D)
	BUILD_128BIT_CONST(val_green , 00,96, 00,96, 00,96, 00,96, 00,96, 00,96, 00,96, 00,96)
	BUILD_128BIT_CONST(val_blue  , 00,4D, 00,4D, 00,4D, 00,4D, 00,4D, 00,4D, 00,4D, 00,4D)

	const __m128i m0 = _mm_load_si128( IS_RGB ? (const __m128i*)mask4 : (const __m128i*)mask0);
	const __m128i m1 = _mm_load_si128( IS_RGB ? (const __m128i*)mask5 : (const __m128i*)mask1);
	const __m128i m2 = _mm_load_si128((const __m128i*)mask2);
	const __m128i m3 = _mm_load_si128((const __m128i*)mask3);
	const __m128i m4 = _mm_load_si128( IS_RGB ? (const __m128i*)mask0 : (const __m128i*)mask4);
	const __m128i m5 = _mm_load_si128( IS_RGB ? (const __m128i*)mask1 : (const __m128i*)mask5);

	const __m128i m6 = _mm_load_si128( IS_RGB ? (const __m128i*)mask10 : (const __m128i*)mask6);
	const __m128i m7 = _mm_load_si128( IS_RGB ? (const __m128i*)mask11 : (const __m128i*)mask7);
	const __m128i m8 = _mm_load_si128((const __m128i*)mask8);
	const __m128i m9 = _mm_load_si128((const __m128i*)mask9);
	const __m128i m10= _mm_load_si128( IS_RGB ? (const __m128i*)mask6 : (const __m128i*)mask10);
	const __m128i m11= _mm_load_si128( IS_RGB ? (const __m128i*)mask7 : (const __m128i*)mask11);

	const __m128i mask_low= _mm_load_si128((const __m128i*)mask_to_low);

	const __m128i VAL_R = _mm_load_si128((const __m128i*)val_red);
	const __m128i VAL_G = _mm_load_si128((const __m128i*)val_green);
	const __m128i VAL_B = _mm_load_si128((const __m128i*)val_blue);

	const int sw = w >> 4;  // This are the number of 3*16 blocks in each row
	const int sh = h ;

	for (int i=0; i<sh; i++)
	{
		for (int j=0; j<sw; j++)
		{
			// We process RGB data in blocks of 3 x 16byte blocks:
			const __m128i d0 = _mm_load_si128((const __m128i*)in); in += 16;
			const __m128i d1 = _mm_load_si128((const __m128i*)in); in += 16;
			const __m128i d2 = _mm_load_si128((const __m128i*)in); in += 16;

			// First 8 bytes of gray levels:
			{
				const __m128i BLUES_0_7  = _mm_or_si128(_mm_shuffle_epi8(d0,m0),_mm_shuffle_epi8(d1,m1));
				const __m128i GREENS_0_7 = _mm_or_si128(_mm_shuffle_epi8(d0,m2),_mm_shuffle_epi8(d1,m3));
				const __m128i REDS_0_7   = _mm_or_si128(_mm_shuffle_epi8(d0,m4),_mm_shuffle_epi8(d1,m5));

				// _mm_mulhi_epu16(): Multiplies the 8 unsigned 16-bit integers from a by the 8 unsigned 16-bit integers from b.
				//r0 := (a0 * b0)[31:16]
				//r1 := (a1 * b1)[31:16]
				//...
				//r7 := (a7 * b7)[31:16]
				//
				const __m128i GRAYS_0_7 =
					_mm_adds_epu16(
						_mm_mulhi_epu16(REDS_0_7,   VAL_R),
					_mm_adds_epu16(
						_mm_mulhi_epu16(GREENS_0_7, VAL_G),
						_mm_mulhi_epu16(BLUES_0_7,  VAL_B)
					));

				_mm_storel_epi64((__m128i*)out, _mm_shuffle_epi8(GRAYS_0_7,mask_low));
				out+=8;
			}

			// Second 8 bytes of gray levels:
			{
				const __m128i BLUES_8_15  = _mm_or_si128(_mm_shuffle_epi8(d1,m6),_mm_shuffle_epi8(d2,m7));
				const __m128i GREENS_8_15 = _mm_or_si128(_mm_shuffle_epi8(d1,m8),_mm_shuffle_epi8(d2,m9));
				const __m128i REDS_8_15   = _mm_or_si128(_mm_shuffle_epi8(d1,m10),_mm_shuffle_epi8(d2,m11));

				const __m128i GRAYS_8_15 =
					_mm_adds_epu16(
						_mm_mulhi_epu16(REDS_8_15,   VAL_R),
					_mm_adds_epu16(
						_mm_mulhi_epu16(GREENS_8_15, VAL_G),
						_mm_mulhi_epu16(BLUES_8_15,  VAL_B)
					));

				_mm_storel_epi64((__m128i*)out, _mm_shuffle_epi8(GRAYS_8_15,mask_low));
				out+=8;
			}
		}
	}

} // end private_image_SSSE3_rgb_or_bgr_to_gray_8u()


/** Convert a RGB image (3cu8) into a GRAYSCALE (1c8u) image, using Y=77*R+150*G+29*B
  *  - <b>Input format:</b> uint8_t, 3 channels (BGR order)
  *  - <b>Output format:</b> uint8_t, 1 channel
  *  - <b>Preconditions:</b> in & out aligned to 16bytes, w = k*16 (w=width in pixels), widthStep=w*3 and w*1
  *  - <b>Notes:</b>
  *  - <b>Requires:</b> SSSE3
  *  - <b>Invoked from:</b> mrpt::utils::CImage::grayscale(), mrpt::utils::CImage::grayscaleInPlace()
  */
void image_SSSE3_bgr_to_gray_8u(const uint8_t* in, uint8_t* out, int w, int h)
{
	private_image_SSSE3_rgb_or_bgr_to_gray_8u<false>(in,out,w,h);
}

/** Convert a RGB image (3cu8) into a GRAYSCALE (1c8u) image, using Y=77*R+150*G+29*B
  *  - <b>Input format:</b> uint8_t, 3 channels (RGB order)
  *  - <b>Output format:</b> uint8_t, 1 channel
  *  - <b>Preconditions:</b> in & out aligned to 16bytes, w = k*16 (w=width in pixels), widthStep=w*3 and w*1
  *  - <b>Notes:</b>
  *  - <b>Requires:</b> SSSE3
  *  - <b>Invoked from:</b> mrpt::utils::CImage::grayscale(), mrpt::utils::CImage::grayscaleInPlace()
  */
void image_SSSE3_rgb_to_gray_8u(const uint8_t* in, uint8_t* out, int w, int h)
{
	private_image_SSSE3_rgb_or_bgr_to_gray_8u<true>(in,out,w,h);
}


/**  @} */

#endif // end of MRPT_HAS_SSE3
