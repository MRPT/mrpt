/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#ifndef mrpt_utils_tpixelcoord_H
#define mrpt_utils_tpixelcoord_H

#include <iosfwd>
#include <mrpt/base/link_pragmas.h>

namespace mrpt
{
	namespace utils
	{
		/** A pair (x,y) of pixel coordinates (subpixel resolution). \ingroup mrpt_base_grp  */
		struct BASE_IMPEXP TPixelCoordf
		{
			/** The type of \a x and \a y */
			typedef float pixel_coord_t; 

			float x,y;

			/** Default constructor: undefined values of x,y */
			TPixelCoordf() : x(),y() {}

			/** Constructor from x,y values */
			TPixelCoordf(const float _x,const float _y) : x(_x), y(_y) { }
		};

		/** Prints TPixelCoordf as "(x,y)" */
		std::ostream BASE_IMPEXP & operator <<(std::ostream& o, const TPixelCoordf& p); 

		/** A pair (x,y) of pixel coordinates (integer resolution). */
		struct BASE_IMPEXP TPixelCoord
		{
			/** The type of \a x and \a y */
			typedef int pixel_coord_t; 

			TPixelCoord() : x(0),y(0) { }
			TPixelCoord(const int _x,const int _y) : x(_x), y(_y) { }

			int x,y;
		};

		/** Prints TPixelCoord as "(x,y)" */
		std::ostream BASE_IMPEXP & operator <<(std::ostream& o, const TPixelCoord& p); 

		/** A type for image sizes. */
		typedef TPixelCoord TImageSize; 

	} // end namespace
}

#endif

