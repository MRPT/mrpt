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

#ifndef mrpt_utils_tpixelcoord_H
#define mrpt_utils_tpixelcoord_H

#include <iostream>
#include <mrpt/base/link_pragmas.h>

namespace mrpt
{
	namespace utils
	{
		/** A pair (x,y) of pixel coordinates (subpixel resolution). \ingroup mrpt_base_grp  */
		struct BASE_IMPEXP TPixelCoordf
		{
			typedef float pixel_coord_t; //!< The type of \a x and \a y

			float x,y;

			/** Default constructor: undefined values of x,y */
			TPixelCoordf() : x(),y() {}

			/** Constructor from x,y values */
			TPixelCoordf(const float _x,const float _y) : x(_x), y(_y) { }
		};

		std::ostream BASE_IMPEXP & operator <<(std::ostream& o, const TPixelCoordf& p); //!< Prints TPixelCoordf as "(x,y)"

		/** A pair (x,y) of pixel coordinates (integer resolution). */
		struct BASE_IMPEXP TPixelCoord
		{
			typedef int pixel_coord_t; //!< The type of \a x and \a y

			TPixelCoord() : x(0),y(0) { }
			TPixelCoord(const int _x,const int _y) : x(_x), y(_y) { }

			int x,y;
		};

		std::ostream BASE_IMPEXP & operator <<(std::ostream& o, const TPixelCoord& p); //!< Prints TPixelCoord as "(x,y)"

		typedef TPixelCoord TImageSize; //!< A type for image sizes.

	} // end namespace
}

#endif

