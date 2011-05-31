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
