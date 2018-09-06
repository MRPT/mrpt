/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <iosfwd>
#include <utility>

namespace mrpt::img
{
/** A pair (x,y) of pixel coordinates (subpixel resolution). \ingroup
 * mrpt_img_grp  */
struct TPixelCoordf
{
	/** The type of \a x and \a y */
	using pixel_coord_t = float;

	float x{.0f}, y{.0f};

	/** Default constructor: undefined values of x,y */
	TPixelCoordf() = default;
	/** Constructor from x,y values */
	TPixelCoordf(const float _x, const float _y) : x(_x), y(_y) {}
	template <typename T>
	TPixelCoordf(const std::pair<T, T>& p) : x(p.first), y(p.second)
	{
	}
};

/** Prints TPixelCoordf as "(x,y)" */
std::ostream& operator<<(std::ostream& o, const TPixelCoordf& p);

/** A pair (x,y) of pixel coordinates (integer resolution). */
struct TPixelCoord
{
	/** The type of \a x and \a y */
	using pixel_coord_t = int;

	TPixelCoord() = default;
	TPixelCoord(const int _x, const int _y) : x(_x), y(_y) {}
	int x{0}, y{0};
};

/** Prints TPixelCoord as "(x,y)" */
std::ostream& operator<<(std::ostream& o, const TPixelCoord& p);

/** A type for image sizes. */
using TImageSize = TPixelCoord;

}  // namespace mrpt::img
