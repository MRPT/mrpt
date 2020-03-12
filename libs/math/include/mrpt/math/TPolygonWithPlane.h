/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/math/TPlane.h>
#include <mrpt/math/TPolygon2D.h>
#include <mrpt/math/TPolygon3D.h>
#include <mrpt/math/TPose3D.h>
#include <vector>

namespace mrpt::math
{
/** \addtogroup geometry_grp
 * @{ */

/** Slightly heavyweight type to speed-up calculations with polygons in 3D
 * \sa TPolygon3D,TPlane
 */
class TPolygonWithPlane
{
   public:
	/** Actual polygon. */
	TPolygon3D poly;
	/** Plane containing the polygon. */
	TPlane plane;
	/** Plane's pose.  \sa inversePose */
	mrpt::math::TPose3D pose;
	/** Plane's inverse pose. \sa pose */
	mrpt::math::TPose3D inversePose;
	/** Polygon, after being projected to the plane using inversePose. \sa
	 * inversePose */
	TPolygon2D poly2D;
	/** Constructor. Takes a polygon and computes each parameter. */
	TPolygonWithPlane(const TPolygon3D& p);
	/** Basic constructor. Needed to create containers  \sa
	 * TPolygonWithPlane(const TPolygon3D &) */
	TPolygonWithPlane() = default;
	/** Static method for vectors. Takes a set of polygons and creates every
	 * TPolygonWithPlane  */
	static void getPlanes(
		const std::vector<TPolygon3D>& oldPolys,
		std::vector<TPolygonWithPlane>& newPolys);
};

/** @} */

}  // namespace mrpt::math
