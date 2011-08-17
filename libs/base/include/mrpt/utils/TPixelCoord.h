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
			TPixelCoord() : x(0),y(0) { }
			TPixelCoord(const int _x,const int _y) : x(_x), y(_y) { }

			int x,y;
		};

		std::ostream BASE_IMPEXP & operator <<(std::ostream& o, const TPixelCoord& p); //!< Prints TPixelCoord as "(x,y)"

		typedef TPixelCoord TImageSize; //!< A type for image sizes.

	} // end namespace
}

#endif

