/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */

#ifndef mrpt_utils_tcolor_H
#define mrpt_utils_tcolor_H

#include <mrpt/utils/mrpt_stdint.h>    // compiler-independent version of "stdint.h"
#include <mrpt/utils/mrpt_inttypes.h>  // compiler-independent version of "inttypes.h"

namespace mrpt
{
	namespace utils
	{

		/** A RGB color - 8bit 
		 * \ingroup mrpt_base_grp */
		struct BASE_IMPEXP TColor
		{
			inline TColor() : R(0),G(0),B(0),A(255) { }
			inline TColor(uint8_t r,uint8_t g,uint8_t b, uint8_t alpha=255) : R(r),G(g),B(b),A(alpha) { }
			inline explicit TColor(const unsigned int color_RGB_24bit) : R(uint8_t(color_RGB_24bit>>16)),G(uint8_t(color_RGB_24bit>>8)),B(uint8_t(color_RGB_24bit)),A(255) { }
			inline TColor(const unsigned int color_RGB_24bit, const uint8_t alpha) : R(uint8_t(color_RGB_24bit>>16)),G(uint8_t(color_RGB_24bit>>8)),B(uint8_t(color_RGB_24bit)),A(alpha) { }
			uint8_t  R,G,B,A;

			/** Operator for implicit conversion into an int binary representation 0xRRGGBB */
			inline operator unsigned int(void) const { return (((unsigned int)R)<<16) | (((unsigned int)G)<<8) | B; }

			static TColor red; //!< Predefined colors
			static TColor green;//!< Predefined colors
			static TColor blue;//!< Predefined colors
			static TColor white;//!< Predefined colors
			static TColor black;//!< Predefined colors
			static TColor gray;	//!< Predefined colors
		};

		/** A RGB color - floats in the range [0,1] 
		 * \ingroup mrpt_base_grp */
		struct BASE_IMPEXP TColorf
		{
			TColorf(float r=0,float g=0,float b=0, float alpha=1.0f) : R(r),G(g),B(b),A(alpha) { }
			explicit TColorf(const TColor &col) : R(col.R*(1.f/255)),G(col.G*(1.f/255)),B(col.B*(1.f/255)),A(col.A*(1.f/255)) { }
			float R,G,B,A;
		};

	} // end namespace
}

#endif

