/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoseOrPoint.h>
#include <vector>

namespace mrpt::math
{
/**
 * 2D polygon, inheriting from std::vector<TPoint2D>.
 * \sa TPolygon3D,TSegment2D,TLine2D,TPoint2D, CPolygon
 * \ingroup geometry_grp
 */
class TPolygon2D : public std::vector<TPoint2D>
{
   public:
	/** Distance to a point (always >=0) */
	double distance(const TPoint2D& point) const;
	/** Check whether a point is inside (or within geometryEpsilon of a polygon
	 * edge). This works for concave or convex polygons. */
	bool contains(const TPoint2D& point) const;
	/** Gets as set of segments, instead of points. */
	void getAsSegmentList(std::vector<TSegment2D>& v) const;
	/** Projects into 3D space, zeroing the z. */
	void generate3DObject(TPolygon3D& p) const;
	/** Polygon's central point. */
	void getCenter(TPoint2D& p) const;
	/** Checks whether is convex. */
	bool isConvex() const;
	/** Erase repeated vertices.  \sa removeRedundantVertices */
	void removeRepeatedVertices();
	/** Erase every redundant vertex from the polygon, saving space. \sa
	 * removeRepeatedVertices */
	void removeRedundantVertices();
	/** Gets plot data, ready to use on a 2D plot. \sa
	 * mrpt::gui::CDisplayWindowPlots */
	void getPlotData(std::vector<double>& x, std::vector<double>& y) const;
	/** Get polygon bounding box. \exception On empty polygon */
	void getBoundingBox(TPoint2D& min_coords, TPoint2D& max_coords) const;
	/** Default constructor  */
	TPolygon2D() : std::vector<TPoint2D>() {}
	/** Constructor for a given number of vertices, intializing them as garbage.
	 */
	explicit TPolygon2D(size_t N) : std::vector<TPoint2D>(N) {}
	/** Implicit constructor from a vector of 2D points */
	TPolygon2D(const std::vector<TPoint2D>& v) : std::vector<TPoint2D>(v) {}
	/** Constructor from a 3D object. */
	explicit TPolygon2D(const TPolygon3D& p);
	/** Static method to create a regular polygon, given its size and radius.
	 * \throw std::logic_error if radius is near zero or the number of edges is
	 * less than three.
	 */
	static void createRegularPolygon(
		size_t numEdges, double radius, TPolygon2D& poly);
	/** Static method to create a regular polygon from its size and radius. The
	 * center will correspond to the given pose.
	 * \throw std::logic_error if radius is near zero or the number of edges is
	 * less than three.
	 */
	static void createRegularPolygon(
		size_t numEdges, double radius, TPolygon2D& poly,
		const mrpt::math::TPose2D& pose);
};

}  // namespace mrpt::math

namespace mrpt::typemeta
{
// Specialization must occur in the same namespace
MRPT_DECLARE_TTYPENAME_NO_NAMESPACE(TPolygon2D, mrpt::math)
}  // namespace mrpt::typemeta
