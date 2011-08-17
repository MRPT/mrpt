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

