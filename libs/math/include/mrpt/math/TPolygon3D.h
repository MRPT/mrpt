/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/containers/yaml_frwd.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPoseOrPoint.h>

#include <vector>

namespace mrpt::math
{
/** 3D polygon, inheriting from std::vector<TPoint3D>
 * \sa TPolygon2D,TSegment3D,TLine3D,TPlane,TPoint3D
 * \ingroup geometry_grp
 */
class TPolygon3D : public std::vector<TPoint3D>
{
   public:
	/** Default constructor. Creates a polygon with no vertices.  */
	TPolygon3D() : std::vector<TPoint3D>() {}

	/** Constructor for a given size. Creates a polygon with a fixed number of
	 * vertices (uninitialized) */
	explicit TPolygon3D(size_t N) : std::vector<TPoint3D>(N) {}

	/** Implicit constructor from a 3D points vector. */
	TPolygon3D(const std::vector<TPoint3D>& v) : std::vector<TPoint3D>(v) {}

	/** Constructor from a 2D object. Zeroes the z. */
	explicit TPolygon3D(const TPolygon2D& p);

	/** Builds a polygon from a YAML sequence (vertices) of sequences (`[x y z]`
	 * coordinates).
	 * \sa asYAML
	 * \note User must include `#include <mrpt/containers/yaml.h>` if using this
	 *       method, only a forward declaration is defined here to speed up
	 *       compilation \note (New in MRPT 2.4.1)
	 */
	static TPolygon3D FromYAML(const mrpt::containers::yaml& c);

	/** Static method to create a regular polygon, given its size and radius.
	 * \throw std::logic_error if number of edges is less than three, or radius
	 * is near zero. */

	/** Constructor from list of vertices data, for example:
	 * \code
	 * const mrpt::math::TPolygon3D p = {
	 *  {-6.0, 0.5, 0.0}, {8.0, 2.0, 1.0}, {10.0, 4.0, 2.0}
	 *  };
	 * \endcode
	 */
	TPolygon3D(const std::initializer_list<TPoint3D>&& vertices)
		: std::vector<TPoint3D>(
			  std::forward<const std::initializer_list<TPoint3D>&>(vertices))
	{
	}

	/** Returns a YAML representation of the polygon as a sequence (vertices) of
	 * sequences (`[x y z]` coordinates).
	 *
	 * \sa FromYAML
	 *
	 * \note User must include `#include <mrpt/containers/yaml.h>` if using this
	 *       method, only a forward declaration is defined here to speed up
	 *       compilation \note (New in MRPT 2.4.1)
	 */
	mrpt::containers::yaml asYAML() const;

	/** Distance to point (always >=0) */
	double distance(const TPoint3D& point) const;
	/** Check whether a point is inside (or within geometryEpsilon of a polygon
	 * edge). This works for concave or convex polygons. */
	bool contains(const TPoint3D& point) const;
	/** Gets as set of segments, instead of set of points. */
	void getAsSegmentList(std::vector<TSegment3D>& v) const;
	/** Gets a plane which contains the polygon. Returns false if the polygon is
	 * skew and cannot be fit inside a plane. */
	bool getPlane(TPlane& p) const;
	/** Gets the best fitting plane, disregarding whether the polygon actually
	 * fits inside or not. \sa getBestFittingPlane */
	void getBestFittingPlane(TPlane& p) const;
	/** Projects into a 2D space, discarding the z. \sa getPlane,isSkew */
	void generate2DObject(TPolygon2D& p) const;
	/** Get polygon's central point. */
	void getCenter(TPoint3D& p) const;
	/** Check whether the polygon is skew. Returns true if there doesn't exist a
	 * plane in which the polygon can fit. \sa getBestFittingPlane */
	bool isSkew() const;
	/** Remove polygon's repeated vertices. */
	void removeRepeatedVertices();
	/** Erase every redundant vertex, thus saving space. */
	void removeRedundantVertices();
	static void createRegularPolygon(
		size_t numEdges, double radius, TPolygon3D& poly);
	/** Static method to create a regular polygon, given its size and radius.
	 * The center will be located on the given pose.
	 * \throw std::logic_error if number of edges is less than three, or radius
	 * is near zero.
	 */
	static void createRegularPolygon(
		size_t numEdges, double radius, TPolygon3D& poly,
		const mrpt::math::TPose3D& pose);
};

/** Text streaming function */
std::ostream& operator<<(std::ostream& o, const TPolygon3D& p);

}  // namespace mrpt::math

namespace mrpt::typemeta
{
// Specialization must occur in the same namespace
MRPT_DECLARE_TTYPENAME_NO_NAMESPACE(TPolygon3D, mrpt::math)

}  // namespace mrpt::typemeta
