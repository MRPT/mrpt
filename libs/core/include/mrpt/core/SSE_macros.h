/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

// Useful macro for masks used in _mm_shuffle_epi8()
#define BUILD_128BIT_CONST(                                                   \
	_name, B0, B1, B2, B3, B4, B5, B6, B7, B8, B9, B10, B11, B12, B13, B14,   \
	B15)                                                                      \
	alignas(MRPT_MAX_ALIGN_BYTES) const __m128i _name = _mm_setr_epi8(        \
		0x##B15, 0x##B14, 0x##B13, 0x##B12, 0x##B11, 0x##B10, 0x##B9, 0x##B8, \
		0x##B7, 0x##B6, 0x##B5, 0x##B4, 0x##B3, 0x##B2, 0x##B1, 0x##B0);

#if defined(_MSC_VER)
#define SSE_DISABLE_WARNINGS \
	_Pragma("warning(push))") _Pragma("warning(disable : 4309)")
#elif defined(__GNUC__)
#define SSE_DISABLE_WARNINGS       \
	_Pragma("GCC diagnostic push") \
		_Pragma("GCC diagnostic ignored \"-Wsign-conversion\"")
#elif defined(__clang__)
#define SSE_DISABLE_WARNINGS         \
	_Pragma("clang diagnostic push") \
		_Pragma("clang diagnostic ignored \"-Wsign-conversion\"")
#endif

#if defined(_MSC_VER)
#define SSE_RESTORE_SIGN_WARNINGS _Pragma("warning(pop)")
#elif defined(__GNUC__)
#define SSE_RESTORE_SIGN_WARNINGS _Pragma("GCC diagnostic pop")
#elif defined(__clang__)
#define SSE_RESTORE_SIGN_WARNINGS _Pragma("clang diagnostic pop")
#endif
