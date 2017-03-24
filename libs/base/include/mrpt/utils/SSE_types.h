/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef mrpt_utils_SSE_types_H
#define mrpt_utils_SSE_types_H

#include <mrpt/config.h>

// SSE2 types:
#if MRPT_HAS_SSE2
    extern "C" {
	#include <emmintrin.h>
	#include <mmintrin.h>
    }
#endif

// SSE3/SSSE3 types:
#if MRPT_HAS_SSE3
    extern "C" {
	#include <pmmintrin.h>
	#if defined(__GNUC__)
		#include <immintrin.h>  // Meta-header
	#endif
    }
#endif

// SSE4.1 types:
#if MRPT_HAS_SSE4_1
# if defined (__SSE4_2__) || defined (__SSE4_1__)
#  include <smmintrin.h>
# endif
#endif


#endif

