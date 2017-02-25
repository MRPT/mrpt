/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef mrpt_utils_sse_macros_H
#define mrpt_utils_sse_macros_H

// Useful macro for masks used in _mm_shuffle_epi8()
#define BUILD_128BIT_CONST(_name, B0,B1,B2,B3,B4,B5,B6,B7,B8,B9,B10,B11,B12,B13,B14,B15)  \
	MRPT_ALIGN16 const unsigned long long _name[2] = { 0x##B7##B6##B5##B4##B3##B2##B1##B0##ull, 0x##B15##B14##B13##B12##B11##B10##B9##B8##ull };


#endif

