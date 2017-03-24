/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CImage_SSEx_H
#define CImage_SSEx_H

#include <mrpt/config.h>

// See documentation in the .cpp files CImage_SSE*.cpp

void image_SSE2_scale_half_1c8u         (const uint8_t* in, uint8_t* out, int w, int h);
void image_SSSE3_scale_half_3c8u        (const uint8_t* in, uint8_t* out, int w, int h);
void image_SSE2_scale_half_smooth_1c8u  (const uint8_t* in, uint8_t* out, int w, int h);
void image_SSSE3_rgb_to_gray_8u         (const uint8_t* in, uint8_t* out, int w, int h);
void image_SSSE3_bgr_to_gray_8u         (const uint8_t* in, uint8_t* out, int w, int h);


#endif
