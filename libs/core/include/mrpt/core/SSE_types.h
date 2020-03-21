/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/config.h>

// SSE2 types:
#if MRPT_HAS_SSE2
extern "C"
{
#include <emmintrin.h>
#include <mmintrin.h>
}
#endif

// SSE3/SSSE3 types:
#if MRPT_HAS_SSE3
extern "C"
{
#include <pmmintrin.h>
#include <tmmintrin.h>
#if defined(__GNUC__)
#include <immintrin.h>  // Meta-header
#endif
}
#endif

// SSE4.1 types:
#if MRPT_HAS_SSE4_1
#if defined(__SSE4_2__) || defined(__SSE4_1__)
#include <smmintrin.h>
#endif
#endif

// Helpers:
#if MRPT_HAS_SSE2
template <bool ALIGNED>
__m128i mm_load_si128(__m128i const* ptr);

template <>
inline __m128i mm_load_si128<true>(__m128i const* ptr)
{
	return _mm_load_si128(ptr);
}

template <>
inline __m128i mm_load_si128<false>(__m128i const* ptr)
{
	return _mm_loadu_si128(ptr);
}

/** Use to check for 2^N multiples `is_multiple<16>(v)`, etc. */
template <int k, typename T>
bool is_multiple(const T val)
{
	return (val & (k - 1)) == 0;
}

#endif
