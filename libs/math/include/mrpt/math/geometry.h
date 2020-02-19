/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CSparseMatrixTemplate.h>
#include <mrpt/math/TLine2D.h>
#include <mrpt/math/TPlane.h>
#include <mrpt/math/TPolygon2D.h>
#include <mrpt/math/TPolygon3D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/epsilon.h>
#include <mrpt/math/math_frwds.h>  // forward declarations
#include <mrpt/math/wrap2pi.h>

namespace mrpt::math
{
/** \addtogroup geometry_grp Geometry: lines, planes, intersections, SLERP,
 * "lightweight" point & pose classes
 *  \ingroup mrpt_math_grp
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

/** @name Simple intersection operations, relying basically on geometrical
   operations.
	@{
 */
/** Gets the intersection between two 3D segments. Possible outcomes:
 *		- Segments intersect: Return=true, obj.getType()=GEOMETRIC_TYPE_POINT
 *		- Segments don't intersect & are parallel: Return=true,
 *obj.getType()=GEOMETRIC_TYPE_SEGMENT, obj is the segment "in between" both
 *segments.
 *		- Segments don't intersect & aren't parallel: Return=false.
 * \sa TObject3D
 */
bool intersect(const TSegment3D& s1, const TSegment3D& s2, TObject3D& obj);

/** Gets the intersection between a 3D segment and a plane. Possible outcomes:
 *		- Don't intersect: Return=false
 *		- s1 is within the plane: Return=true,
 *obj.getType()=GEOMETRIC_TYPE_SEGMENT
 *		- s1 intersects the plane at one point: Return=true,
 *obj.getType()=GEOMETRIC_TYPE_POINT
 * \sa TObject3D
 */
bool intersect(const TSegment3D& s1, const TPlane& p2, TObject3D& obj);

/** Gets the intersection between a 3D segment and a 3D line. Possible outcomes:
 *		- They don't intersect : Return=false
 *		- s1 lies within the line: Return=true,
 *obj.getType()=GEOMETRIC_TYPE_SEGMENT
 *		- s1 intersects the line at a point: Return=true,
 *obj.getType()=GEOMETRIC_TYPE_POINT
 * \sa TObject3D
 */
bool intersect(const TSegment3D& s1, const TLine3D& r2, TObject3D& obj);

/** Gets the intersection between a plane and a 3D segment. Possible outcomes:
 *		- Don't intersect: Return=false
 *		- s2 is within the plane: Return=true,
 *obj.getType()=GEOMETRIC_TYPE_SEGMENT
 *		- s2 intersects the plane at one point: Return=true,
 *obj.getType()=GEOMETRIC_TYPE_POINT
 * \sa TObject3D
 */
inline bool intersect(const TPlane& p1, const TSegment3D& s2, TObject3D& obj)
{
	return intersect(s2, p1, obj);
}

/** Gets the intersection between two planes. Possible outcomes:
 *		- Planes are parallel: Return=false
 *		- Planes intersect into a line: Return=true,
 *obj.getType()=GEOMETRIC_TYPE_LINE
 * \sa TObject3D
 */
bool intersect(const TPlane& p1, const TPlane& p2, TObject3D& obj);

/** Gets the intersection between a plane and a 3D line. Possible outcomes:
 *		- Line is parallel to plane but not within it: Return=false
 *		- Line is contained in the plane: Return=true,
 *obj.getType()=GEOMETRIC_TYPE_LINE
 *		- Line intersects the plane at one point: Return=true,
 *obj.getType()=GEOMETRIC_TYPE_POINT
 * \sa TObject3D
 */
bool intersect(const TPlane& p1, const TLine3D& p2, TObject3D& obj);

/** Gets the intersection between a 3D line and a 3D segment. Possible outcomes:
 *		- They don't intersect : Return=false
 *		- s2 lies within the line: Return=true,
 *obj.getType()=GEOMETRIC_TYPE_SEGMENT
 *		- s2 intersects the line at a point: Return=true,
 *obj.getType()=GEOMETRIC_TYPE_POINT
 * \sa TObject3D
 */
inline bool intersect(const TLine3D& r1, const TSegment3D& s2, TObject3D& obj)
{
	return intersect(s2, r1, obj);
}

/** Gets the intersection between a 3D line and a plane. Possible outcomes:
 *		- Line is parallel to plane but not within it: Return=false
 *		- Line is contained in the plane: Return=true,
 *obj.getType()=GEOMETRIC_TYPE_LINE
 *		- Line intersects the plane at one point: Return=true,
 *obj.getType()=GEOMETRIC_TYPE_POINT
 * \sa TObject3D
 */
inline bool intersect(const TLine3D& r1, const TPlane& p2, TObject3D& obj)
{
	return intersect(p2, r1, obj);
}

/** Gets the intersection between two 3D lines. Possible outcomes:
 *		- Lines do not intersect: Return=false
 *		- Lines are parallel and do not coincide: Return=false
 *		- Lines coincide (are the same): Return=true,
 *obj.getType()=GEOMETRIC_TYPE_LINE
 *		- Lines intesect in a point: Return=true,
 *obj.getType()=GEOMETRIC_TYPE_POINT
 * \sa TObject3D
 */
bool intersect(const TLine3D& r1, const TLine3D& r2, TObject3D& obj);

/** Gets the intersection between two 2D lines. Possible outcomes:
 *		- Lines do not intersect: Return=false
 *		- Lines are parallel and do not coincide: Return=false
 *		- Lines coincide (are the same): Return=true,
 *obj.getType()=GEOMETRIC_TYPE_LINE
 *		- Lines intesect in a point: Return=true,
 *obj.getType()=GEOMETRIC_TYPE_POINT
 * \sa TObject2D
 */
bool intersect(const TLine2D& r1, const TLine2D& r2, TObject2D& obj);

/** Gets the intersection between a 2D line and a 2D segment. Possible outcomes:
 *		- They don't intersect: Return=false
 *		- s2 lies within the line: Return=true,
 *obj.getType()=GEOMETRIC_TYPE_SEGMENT
 *		- Both intersects in one point: Return=true,
 *obj.getType()=GEOMETRIC_TYPE_POINT
 * \sa TObject2D
 */
bool intersect(const TLine2D& r1, const TSegment2D& s2, TObject2D& obj);

/** Gets the intersection between a 2D line and a 2D segment. Possible outcomes:
 *		- They don't intersect: Return=false
 *		- s1 lies within the line: Return=true,
 *obj.getType()=GEOMETRIC_TYPE_SEGMENT
 *		- Both intersects in one point: Return=true,
 *obj.getType()=GEOMETRIC_TYPE_POINT
 * \sa TObject2D
 */
inline bool intersect(const TSegment2D& s1, const TLine2D& r2, TObject2D& obj)
{
	return intersect(r2, s1, obj);
}

/** Gets the intersection between two 2D segments. Possible outcomes:
 *		- Segments intersect: Return=true, obj.getType()=GEOMETRIC_TYPE_POINT
 *		- Segments don't intersect & are parallel: Return=true,
 *obj.getType()=GEOMETRIC_TYPE_SEGMENT, obj is the segment "in between" both
 *segments.
 *		- Segments don't intersect & aren't parallel: Return=false.
 * \sa TObject2D
 */
bool intersect(const TSegment2D& s1, const TSegment2D& s2, TObject2D& obj);

/** @}
 */

/** @name Angle retrieval methods. Methods which use TSegments will
   automatically use TLines' implicit constructors.
	@{
 */
/**
 * Computes the angle between two planes.
 */
double getAngle(const TPlane& p1, const TPlane& p2);
/**
 * Computes the angle between a plane and a 3D line or segment (implicit
 * constructor will be used if passing a segment instead of a line).
 */
double getAngle(const TPlane& p1, const TLine3D& r2);
/**
 * Computes the angle between a 3D line or segment and a plane (implicit
 * constructor will be used if passing a segment instead of a line).
 */
inline double getAngle(const TLine3D& r1, const TPlane& p2)
{
	return getAngle(p2, r1);
}
/**
 * Computes the accute relative angle (range: [-PI/2,PI/2]) between two lines.
 * \note Implicit constructor allows passing a segment as argument too.
 */
double getAngle(const TLine3D& r1, const TLine3D& r2);
/**
 * Computes the relative angle (range: [-PI,PI]) of line 2 wrt line 1.
 * \note Implicit constructor allows passing a segment as argument too.
 */
double getAngle(const TLine2D& r1, const TLine2D& r2);
/** @}
 */

/** @name Creation of lines from poses.
	@{
 */
/**
 * Gets a 3D line corresponding to the X axis in a given pose. An implicit
 * constructor is used if a TPose3D is given.
 * \sa createFromPoseY,createFromPoseZ,createFromPoseAndVector
 */
void createFromPoseX(const mrpt::math::TPose3D& p, TLine3D& r);
/**
 * Gets a 3D line corresponding to the Y axis in a given pose. An implicit
 * constructor is used if a TPose3D is given.
 * \sa createFromPoseX,createFromPoseZ,createFromPoseAndVector
 */
void createFromPoseY(const mrpt::math::TPose3D& p, TLine3D& r);
/**
 * Gets a 3D line corresponding to the Z axis in a given pose. An implicit
 * constructor is used if a TPose3D is given.
 * \sa createFromPoseX,createFromPoseY,createFromPoseAndVector
 */
void createFromPoseZ(const mrpt::math::TPose3D& p, TLine3D& r);
/**
 * Gets a 3D line corresponding to any arbitrary vector, in the base given by
 * the pose. An implicit constructor is used if a TPose3D is given.
 * \sa createFromPoseX,createFromPoseY,createFromPoseZ
 */
void createFromPoseAndVector(
	const mrpt::math::TPose3D& p, const double (&vector)[3], TLine3D& r);
/**
 * Gets a 2D line corresponding to the X axis in a given pose. An implicit
 * constructor is used if a CPose2D is given.
 * \sa createFromPoseY,createFromPoseAndVector
 */
void createFromPoseX(const TPose2D& p, TLine2D& r);
/**
 * Gets a 2D line corresponding to the Y axis in a given pose. An implicit
 * constructor is used if a CPose2D is given.
 * \sa createFromPoseX,createFromPoseAndVector
 */
void createFromPoseY(const TPose2D& p, TLine2D& r);
/**
 * Gets a 2D line corresponding to any arbitrary vector, in the base given the
 * given pose. An implicit constructor is used if a CPose2D is given.
 * \sa createFromPoseY,createFromPoseAndVector
 */
void createFromPoseAndVector(
	const TPose2D& p, const double (&vector)[2], TLine2D& r);
/** @}
 */

/** @name Other line or plane related methods.
	@{
 */
/**
 * Checks whether this polygon or set of points acceptably fits a plane.
 * \sa TPolygon3D,getEpsilon
 */
bool conformAPlane(const std::vector<TPoint3D>& points);
/**
 * Checks whether this polygon or set of points acceptably fits a plane, and if
 * it's the case returns it in the second argument.
 * \sa TPolygon3D,getEpsilon
 */
bool conformAPlane(const std::vector<TPoint3D>& points, TPlane& p);
/**
 * Checks whether this set of points acceptably fits a 2D line.
 * \sa getEpsilon
 */
bool areAligned(const std::vector<TPoint2D>& points);
/**
 * Checks whether this set of points acceptably fits a 2D line, and if it's the
 * case returns it in the second argument.
 * \sa getEpsilon
 */
bool areAligned(const std::vector<TPoint2D>& points, TLine2D& r);
/**
 * Checks whether this set of points acceptably fits a 3D line.
 * \sa getEpsilon
 */
bool areAligned(const std::vector<TPoint3D>& points);
/**
 * Checks whether this set of points acceptably fits a 3D line, and if it's the
 * case returns it in the second argument.
 */
bool areAligned(const std::vector<TPoint3D>& points, TLine3D& r);
/** @}
 */

/** @name Projections
	@{
 */
/** Uses the given pose 3D to project a point into a new base */
inline void project3D(
	const TPoint3D& point, const mrpt::math::TPose3D& newXYpose,
	TPoint3D& newPoint)
{
	newXYpose.composePoint(point, newPoint);
}
/** Uses the given pose 3D to project a segment into a new base  */
inline void project3D(
	const TSegment3D& segment, const mrpt::math::TPose3D& newXYpose,
	TSegment3D& newSegment)
{
	project3D(segment.point1, newXYpose, newSegment.point1);
	project3D(segment.point2, newXYpose, newSegment.point2);
}

/** Uses the given pose 3D to project a line into a new base */
void project3D(
	const TLine3D& line, const mrpt::math::TPose3D& newXYpose,
	TLine3D& newLine);
/** Uses the given pose 3D to project a plane into a new base */
void project3D(
	const TPlane& plane, const mrpt::math::TPose3D& newXYpose,
	TPlane& newPlane);
/** Uses the given pose 3D to project a polygon into a new base */
void project3D(
	const TPolygon3D& polygon, const mrpt::math::TPose3D& newXYpose,
	TPolygon3D& newPolygon);
/** Uses the given pose 3D to project any 3D object into a new base. */
void project3D(
	const TObject3D& object, const mrpt::math::TPose3D& newXYPose,
	TObject3D& newObject);

/** Projects any 3D object into the plane's base, using its inverse pose. If the
 * object is exactly inside the plane, this projection will zero its Z
 * coordinates */
template <class T>
void project3D(const T& obj, const TPlane& newXYPlane, T& newObj)
{
	mrpt::math::TPose3D pose;
	TPlane(newXYPlane).getAsPose3D(pose);
	project3D(obj, -pose, newObj);
}

/** Projects any 3D object into the plane's base, using its inverse pose and
 * forcing the position of the new coordinates origin. If the object is exactly
 * inside the plane, this projection will zero its Z coordinates  */
template <class T>
void project3D(
	const T& obj, const TPlane& newXYPlane, const TPoint3D& newOrigin,
	T& newObj)
{
	mrpt::math::TPose3D pose;
	// TPlane(newXYPlane).getAsPose3DForcingOrigin(newOrigin,pose);
	TPlane(newXYPlane).getAsPose3D(pose);
	project3D(obj, -pose, newObj);
}

/** Projects a set of 3D objects into the plane's base. */
template <class T>
void project3D(
	const std::vector<T>& objs, const mrpt::math::TPose3D& newXYpose,
	std::vector<T>& newObjs)
{
	size_t N = objs.size();
	newObjs.resize(N);
	for (size_t i = 0; i < N; i++) project3D(objs[i], newXYpose, newObjs[i]);
}

/** Uses the given pose 2D to project a point into a new base. */
void project2D(
	const TPoint2D& point, const TPose2D& newXpose, TPoint2D& newPoint);

/** Uses the given pose 2D to project a segment into a new base  */
inline void project2D(
	const TSegment2D& segment, const TPose2D& newXpose, TSegment2D& newSegment)
{
	project2D(segment.point1, newXpose, newSegment.point1);
	project2D(segment.point2, newXpose, newSegment.point2);
}

/** Uses the given pose 2D to project a line into a new base */
void project2D(const TLine2D& line, const TPose2D& newXpose, TLine2D& newLine);
/** Uses the given pose 2D to project a polygon into a new base. */
void project2D(
	const TPolygon2D& polygon, const TPose2D& newXpose, TPolygon2D& newPolygon);
/** Uses the given pose 2D to project any 2D object into a new base */
void project2D(
	const TObject2D& object, const TPose2D& newXpose, TObject2D& newObject);

/** Projects any 2D object into the line's base, using its inverse pose. If the
 * object is exactly inside the line, this projection will zero its Y
 * coordinate.
 * \tparam CPOSE2D set to TPose2D
 */
template <class T, class CPOSE2D>
void project2D(const T& obj, const TLine2D& newXLine, T& newObj)
{
	CPOSE2D pose;
	newXLine.getAsPose2D(pose);
	project2D(obj, CPOSE2D(0, 0, 0) - pose, newObj);
}

/** Projects any 2D object into the line's base, using its inverse pose and
 * forcing the position of the new coordinate origin. If the object is exactly
 * inside the line, this projection will zero its Y coordinate.
 * \tparam CPOSE2D set to TPose2D
 */
template <class T, class CPOSE2D>
void project2D(
	const T& obj, const TLine2D& newXLine, const TPoint2D& newOrigin, T& newObj)
{
	CPOSE2D pose;
	newXLine.getAsPose2DForcingOrigin(newOrigin, pose);
	project2D(obj, CPOSE2D(0, 0, 0) - pose, newObj);
}

/** Projects a set of 2D objects into the line's base */
template <class T>
void project2D(
	const std::vector<T>& objs, const TPose2D& newXpose,
	std::vector<T>& newObjs)
{
	size_t N = objs.size();
	newObjs.resize(N);
	for (size_t i = 0; i < N; i++) project2D(objs[i], newXpose, newObjs[i]);
}
/** @}
 */

/** @name Polygon intersections. These operations rely more on spatial reasoning
   than in raw numerical operations.
	@{
 */
/** Gets the intersection between a 2D polygon and a 2D segment. \sa TObject2D
 */
bool intersect(const TPolygon2D& p1, const TSegment2D& s2, TObject2D& obj);
/** Gets the intersection between a 2D polygon and a 2D line. \sa TObject2D  */
bool intersect(const TPolygon2D& p1, const TLine2D& r2, TObject2D& obj);
/** Gets the intersection between two 2D polygons. \sa TObject2D */
bool intersect(const TPolygon2D& p1, const TPolygon2D& p2, TObject2D& obj);
/** Gets the intersection between a 2D segment and a 2D polygon.  \sa TObject2D
 */
inline bool intersect(
	const TSegment2D& s1, const TPolygon2D& p2, TObject2D& obj)
{
	return intersect(p2, s1, obj);
}
/** Gets the intersection between a 2D line and a 2D polygon.\sa TObject2D */
inline bool intersect(const TLine2D& r1, const TPolygon2D& p2, TObject2D& obj)
{
	return intersect(p2, r1, obj);
}
/** Gets the intersection between a 3D polygon and a 3D segment. \sa TObject3D
 */
bool intersect(const TPolygon3D& p1, const TSegment3D& s2, TObject3D& obj);
/** Gets the intersection between a 3D polygon and a 3D line. \sa TObject3D  */
bool intersect(const TPolygon3D& p1, const TLine3D& r2, TObject3D& obj);
/** Gets the intersection between a 3D polygon and a plane. \sa TObject3D */
bool intersect(const TPolygon3D& p1, const TPlane& p2, TObject3D& obj);
/** Gets the intersection between two 3D polygons. \sa TObject3D */
bool intersect(const TPolygon3D& p1, const TPolygon3D& p2, TObject3D& obj);
/** Gets the intersection between a 3D segment and a 3D polygon. \sa TObject3D
 */
inline bool intersect(
	const TSegment3D& s1, const TPolygon3D& p2, TObject3D& obj)
{
	return intersect(p2, s1, obj);
}
/** Gets the intersection between a 3D line and a 3D polygon.\sa TObject3D  */
inline bool intersect(const TLine3D& r1, const TPolygon3D& p2, TObject3D& obj)
{
	return intersect(p2, r1, obj);
}
/** Gets the intersection between a plane and a 3D polygon. \sa TObject3D */
inline bool intersect(const TPlane& p1, const TPolygon3D& p2, TObject3D& obj)
{
	return intersect(p2, p1, obj);
}

/** Gets the intersection between two sets of 3D polygons. The intersection is
 * returned as an sparse matrix with each pair of polygons' intersections, and
 * the return value is the amount of intersections found.
 * \sa TObject3D,CSparseMatrixTemplate */
size_t intersect(
	const std::vector<TPolygon3D>& v1, const std::vector<TPolygon3D>& v2,
	CSparseMatrixTemplate<TObject3D>& objs);
/** Gets the intersection between two sets of 3D polygons. The intersection is
 * returned as a vector with every intersection found, and the return value is
 * the amount of intersections found.
 * \sa TObject3D */
size_t intersect(
	const std::vector<TPolygon3D>& v1, const std::vector<TPolygon3D>& v2,
	std::vector<TObject3D>& objs);
/** @}
 */

/** @name Other intersections
	@{
 */
/** Gets the intersection between vectors of geometric objects and returns it in
 * a sparse matrix of either TObject2D or TObject3D.
 * \sa TObject2D,TObject3D,CSparseMatrix */
template <class T, class U, class O>
size_t intersect(
	const std::vector<T>& v1, const std::vector<U>& v2,
	CSparseMatrixTemplate<O>& objs)
{
	size_t M = v1.size(), N = v2.size();
	O obj;
	objs.clear();
	objs.resize(M, N);
	for (size_t i = 0; i < M; i++)
		for (size_t j = 0; j < M; j++)
			if (intersect(v1[i], v2[j], obj)) objs(i, j) = obj;
	return objs.getNonNullElements();
}

/** Gets the intersection between vectors of geometric objects and returns it in
 * a vector of either TObject2D or TObject3D.
 * \sa TObject2D,TObject3D */
template <class T, class U, class O>
size_t intersect(
	const std::vector<T>& v1, const std::vector<U>& v2, std::vector<O> objs)
{
	objs.resize(0);
	O obj;
	for (typename std::vector<T>::const_iterator it1 = v1.begin();
		 it1 != v1.end(); ++it1)
	{
		const T& elem1 = *it1;
		for (typename std::vector<U>::const_iterator it2 = v2.begin();
			 it2 != v2.end(); ++it2)
			if (intersect(elem1, *it2, obj)) objs.push_back(obj);
	}
	return objs.size();
}

/** Gets the intersection between any pair of 2D objects.*/
bool intersect(const TObject2D& o1, const TObject2D& o2, TObject2D& obj);
/** Gets the intersection between any pair of 3D objects.*/
bool intersect(const TObject3D& o1, const TObject3D& o2, TObject3D& obj);
/** @}
 */

/** @name Distances
	@{
 */
/** Gets the distance between two points in a 2D space. */
double distance(const TPoint2D& p1, const TPoint2D& p2);
/** Gets the distance between two points in a 3D space. */
double distance(const TPoint3D& p1, const TPoint3D& p2);
/**  Gets the distance between two lines in a 2D space. */
double distance(const TLine2D& r1, const TLine2D& r2);
/**  Gets the distance between two lines in a 3D space. */
double distance(const TLine3D& r1, const TLine3D& r2);
/**  Gets the distance between two planes. It will be zero if the planes are not
 * parallel. */
double distance(const TPlane& p1, const TPlane& p2);
/** Gets the distance between two polygons in a 2D space. */
double distance(const TPolygon2D& p1, const TPolygon2D& p2);
/** Gets the distance between a polygon and a segment in a 2D space. */
double distance(const TPolygon2D& p1, const TSegment2D& s2);
/** Gets the distance between a segment and a polygon in a 2D space. */
inline double distance(const TSegment2D& s1, const TPolygon2D& p2)
{
	return distance(p2, s1);
}
/** Gets the distance between a polygon and a line in a 2D space. */
double distance(const TPolygon2D& p1, const TLine2D& l2);
inline double distance(const TLine2D& l1, const TPolygon2D& p2)
{
	return distance(p2, l1);
}
/** Gets the distance between two polygons in a 3D space. */
double distance(const TPolygon3D& p1, const TPolygon3D& p2);
/** Gets the distance between a polygon and a segment in a 3D space. */
double distance(const TPolygon3D& p1, const TSegment3D& s2);
/** Gets the distance between a segment and a polygon in a 3D space.*/
inline double distance(const TSegment3D& s1, const TPolygon3D& p2)
{
	return distance(p2, s1);
}
/** Gets the distance between a polygon and a line in a 3D space. */
double distance(const TPolygon3D& p1, const TLine3D& l2);
/** Gets the distance between a line and a polygon in a 3D space */
inline double distance(const TLine3D& l1, const TPolygon3D& p2)
{
	return distance(p2, l1);
}
/** Gets the distance between a polygon and a plane. */
double distance(const TPolygon3D& po, const TPlane& pl);
/** Gets the distance between a plane and a polygon.*/
inline double distance(const TPlane& pl, const TPolygon3D& po)
{
	return distance(po, pl);
}
/** @}
 */

/** @name Bound checkers
	@{
 */
/** Gets the rectangular bounds of a 2D polygon or set of 2D points */
void getRectangleBounds(
	const std::vector<TPoint2D>& poly, TPoint2D& pMin, TPoint2D& pMax);
/** Gets the prism bounds of a 3D polygon or set of 3D points. */
void getPrismBounds(
	const std::vector<TPoint3D>& poly, TPoint3D& pMin, TPoint3D& pMax);
/** @}
 */

/** @name Creation of planes from poses
	@{
 */
/**
 * Given a pose, creates a plane orthogonal to its Z vector.
 * \sa createPlaneFromPoseXZ,createPlaneFromPoseYZ,createPlaneFromPoseAndNormal
 */
void createPlaneFromPoseXY(const mrpt::math::TPose3D& pose, TPlane& plane);
/**
 * Given a pose, creates a plane orthogonal to its Y vector.
 * \sa createPlaneFromPoseXY,createPlaneFromPoseYZ,createPlaneFromPoseAndNormal
 */
void createPlaneFromPoseXZ(const mrpt::math::TPose3D& pose, TPlane& plane);
/**
 * Given a pose, creates a plane orthogonal to its X vector.
 * \sa createPlaneFromPoseXY,createPlaneFromPoseXZ,createPlaneFromPoseAndNormal
 */
void createPlaneFromPoseYZ(const mrpt::math::TPose3D& pose, TPlane& plane);
/**
 * Given a pose and any vector, creates a plane orthogonal to that vector in
 * the pose's coordinates.
 * \sa createPlaneFromPoseXY,createPlaneFromPoseXZ,createPlaneFromPoseYZ
 */
void createPlaneFromPoseAndNormal(
	const mrpt::math::TPose3D& pose, const double (&normal)[3], TPlane& plane);
/**
 * Creates a homogeneus matrix (4x4) such that the coordinate given (0 for x, 1
 * for y, 2 for z) corresponds to the provided vector.
 * \param[in] vec must be a *unitary* vector
 * \sa generateAxisBaseFromDirectionAndAxis()
 */
CMatrixDouble44 generateAxisBaseFromDirectionAndAxis(
	const mrpt::math::TVector3D& vec, uint8_t coord);
/** @}
 */

/** @name Linear regression methods
	@{
 */
/**
 * Using eigenvalues, gets the best fitting line for a set of 2D points.
 * Returns an estimation of the error.
 * \sa spline, leastSquareLinearFit
 */
double getRegressionLine(const std::vector<TPoint2D>& points, TLine2D& line);
/**
 * Using eigenvalues, gets the best fitting line for a set of 3D points.
 * Returns an estimation of the error.
 * \sa spline, leastSquareLinearFit
 */
double getRegressionLine(const std::vector<TPoint3D>& points, TLine3D& line);
/**
 * Using eigenvalues, gets the best fitting plane for a set of 3D points.
 * Returns an estimation of the error.
 * \sa spline, leastSquareLinearFit
 */
double getRegressionPlane(const std::vector<TPoint3D>& points, TPlane& plane);
/** @}
 */

/** @name Miscellaneous Geometry methods
	@{
 */
/**
 * Tries to assemble a set of segments into a set of closed polygons.
 */
void assemblePolygons(
	const std::vector<TSegment3D>& segms, std::vector<TPolygon3D>& polys);
/**
 * Tries to assemble a set of segments into a set of closed polygons, returning
 * the unused segments as another out parameter.
 */
void assemblePolygons(
	const std::vector<TSegment3D>& segms, std::vector<TPolygon3D>& polys,
	std::vector<TSegment3D>& remainder);
/**
 * Extracts all the polygons, including those formed from segments, from the
 * set of objects.
 */
void assemblePolygons(
	const std::vector<TObject3D>& objs, std::vector<TPolygon3D>& polys);
/**
 * Extracts all the polygons, including those formed from segments, from the
 * set of objects.
 */
void assemblePolygons(
	const std::vector<TObject3D>& objs, std::vector<TPolygon3D>& polys,
	std::vector<TObject3D>& remainder);
/**
 * Extracts all the polygons, including those formed from segments, from the
 * set of objects.
 */
void assemblePolygons(
	const std::vector<TObject3D>& objs, std::vector<TPolygon3D>& polys,
	std::vector<TSegment3D>& remainder1, std::vector<TObject3D>& remainder2);

/**
 * Splits a 2D polygon into convex components.
 */
bool splitInConvexComponents(
	const TPolygon2D& poly, std::vector<TPolygon2D>& components);
/**
 * Splits a 3D polygon into convex components.
 * \throw std::logic_error if the polygon can't be fit into a plane.
 */
bool splitInConvexComponents(
	const TPolygon3D& poly, std::vector<TPolygon3D>& components);

/**
 * Gets the bisector of a 2D segment.
 */
void getSegmentBisector(const TSegment2D& sgm, TLine2D& bis);
/**
 * Gets the bisector of a 3D segment.
 */
void getSegmentBisector(const TSegment3D& sgm, TPlane& bis);
/**
 * Gets the bisector of two lines or segments (implicit constructor will be
 * used if necessary)
 */
void getAngleBisector(const TLine2D& l1, const TLine2D& l2, TLine2D& bis);
/**
 * Gets the bisector of two lines or segments (implicit constructor will be
 * used if necessary)
 * \throw std::logic_error if the lines do not fit in a single plane.
 */
void getAngleBisector(const TLine3D& l1, const TLine3D& l2, TLine3D& bis);

/**
 * Fast ray tracing method using polygons' properties.
 * \sa CRenderizable::rayTrace
 */
bool traceRay(
	const std::vector<TPolygonWithPlane>& vec, const mrpt::math::TPose3D& pose,
	double& dist);
/**
 * Fast ray tracing method using polygons' properties.
 * \sa CRenderizable::rayTrace
 */
inline bool traceRay(
	const std::vector<TPolygon3D>& vec, const mrpt::math::TPose3D& pose,
	double& dist)
{
	std::vector<TPolygonWithPlane> pwp;
	TPolygonWithPlane::getPlanes(vec, pwp);
	return traceRay(pwp, pose, dist);
}

/** Computes the cross product of two 3D vectors, returning a vector normal to
  both.
  *  It uses the simple implementation:

	\f[  v_out = \left(
			\begin{array}{c c c}
			\hat{i} ~ \hat{j} ~ \hat{k} \\
			x0 ~ y0 ~ z0 \\
			x1 ~ y1 ~ z1 \\
			\end{array} \right)
	\f]
  */
template <class T, class U, class V>
inline void crossProduct3D(const T& v0, const U& v1, V& vOut)
{
	vOut[0] = v0[1] * v1[2] - v0[2] * v1[1];
	vOut[1] = v0[2] * v1[0] - v0[0] * v1[2];
	vOut[2] = v0[0] * v1[1] - v0[1] * v1[0];
}

//! \overload
template <class T>
inline void crossProduct3D(
	const std::vector<T>& v0, const std::vector<T>& v1, std::vector<T>& v_out)
{
	ASSERT_(v0.size() == 3);
	ASSERT_(v1.size() == 3);
	v_out.resize(3);
	crossProduct3D<std::vector<T>, std::vector<T>, std::vector<T>>(
		v0, v1, v_out);
}
//! overload (returning a vector of size 3 by value).
template <class VEC1, class VEC2>
inline VEC1 crossProduct3D(const VEC1& v0, const VEC2& v1)
{
	VEC1 vOut;
	crossProduct3D<VEC1, VEC2, VEC1>(v0, v1, vOut);
	return vOut;
}

/** Computes the 3x3 skew symmetric matrix from a 3-vector or 3-array:
 * \f[  M([x ~ y ~ z]^\top) = \left(
 * 	\begin{array}{c c c}
 * 	0 & -z & y \\
 * 	z & 0 & -x \\
 * 	-y & x & 0
 * 	\end{array} \right)
 * \f]
 */
template <class VECTOR, class MATRIX>
inline void skew_symmetric3(const VECTOR& v, MATRIX& M)
{
	ASSERT_(v.size() == 3);
	M.setSize(3, 3);
	M(0, 0) = 0;
	M(0, 1) = -v[2];
	M(0, 2) = v[1];
	M(1, 0) = v[2];
	M(1, 1) = 0;
	M(1, 2) = -v[0];
	M(2, 0) = -v[1];
	M(2, 1) = v[0];
	M(2, 2) = 0;
}
//! \overload
template <class VECTOR>
inline mrpt::math::CMatrixDouble33 skew_symmetric3(const VECTOR& v)
{
	mrpt::math::CMatrixDouble33 M(mrpt::math::UNINITIALIZED_MATRIX);
	skew_symmetric3(v, M);
	return M;
}

/** Computes the negative version of a 3x3 skew symmetric matrix from a 3-vector
 * or 3-array:
 * \f[  -M([x ~ y ~ z]^\top) = \left(
 * 	\begin{array}{c c c}
 * 	0 & z & -y \\
 * 	-z & 0 & x \\
 * 	y & -x & 0
 * 	\end{array} \right)
 * \f]
 */
template <class VECTOR, class MATRIX>
inline void skew_symmetric3_neg(const VECTOR& v, MATRIX& M)
{
	ASSERT_(v.size() == 3);
	ASSERT_(M.rows() == 3 && M.cols() == 3);
	M(0, 0) = 0;
	M(0, 1) = v[2];
	M(0, 2) = -v[1];
	M(1, 0) = -v[2];
	M(1, 1) = 0;
	M(1, 2) = v[0];
	M(2, 0) = v[1];
	M(2, 1) = -v[0];
	M(2, 2) = 0;
}
//! \overload
template <class VECTOR>
inline mrpt::math::CMatrixDouble33 skew_symmetric3_neg(const VECTOR& v)
{
	mrpt::math::CMatrixDouble33 M(mrpt::math::UNINITIALIZED_MATRIX);
	skew_symmetric3_neg(v, M);
	return M;
}

/**
 * Returns true if two 2D vectors are parallel. The arguments may be points,
 * arrays, etc.
 */
template <class T, class U>
inline bool vectorsAreParallel2D(const T& v1, const U& v2)
{
	return std::abs(v1[0] * v2[1] - v2[0] * v1[1]) < getEpsilon();
}

/**
 * Returns true if two 3D vectors are parallel. The arguments may be points,
 * arrays, etc.
 */
template <class T, class U>
inline bool vectorsAreParallel3D(const T& v1, const U& v2)
{
	if (std::abs(v1[0] * v2[1] - v2[0] * v1[1]) >= getEpsilon()) return false;
	if (std::abs(v1[1] * v2[2] - v2[1] * v1[2]) >= getEpsilon()) return false;
	return std::abs(v1[2] * v2[0] - v2[2] * v1[0]) < getEpsilon();
}

/** Computes the closest point from a given point to a segment.
 * \sa closestFromPointToLine
 */
void closestFromPointToSegment(
	double Px, double Py, double x1, double y1, double x2, double y2,
	double& out_x, double& out_y);

/** Computes the closest point from a given point to a (infinite) line.
 * \sa closestFromPointToSegment
 */
void closestFromPointToLine(
	double Px, double Py, double x1, double y1, double x2, double y2,
	double& out_x, double& out_y);

/** Returns the square distance from a point to a line.
 */
double closestSquareDistanceFromPointToLine(
	double Px, double Py, double x1, double y1, double x2, double y2);

/** Returns the distance between 2 points in 2D. */
template <typename T>
T distanceBetweenPoints(const T x1, const T y1, const T x2, const T y2)
{
	return std::sqrt(square(x1 - x2) + square(y1 - y2));
}

/** Returns the distance between 2 points in 3D. */
template <typename T>
T distanceBetweenPoints(
	const T x1, const T y1, const T z1, const T x2, const T y2, const T z2)
{
	return std::sqrt(square(x1 - x2) + square(y1 - y2) + square(z1 - z2));
}

/** Returns the square distance between 2 points in 2D. */
template <typename T>
T distanceSqrBetweenPoints(const T x1, const T y1, const T x2, const T y2)
{
	return square(x1 - x2) + square(y1 - y2);
}

/** Returns the square distance between 2 points in 3D. */
template <typename T>
T distanceSqrBetweenPoints(
	const T x1, const T y1, const T z1, const T x2, const T y2, const T z2)
{
	return square(x1 - x2) + square(y1 - y2) + square(z1 - z2);
}

/** Computes the closest point from a given point to a segment, and returns that
 * minimum distance.
 */
template <typename T>
double minimumDistanceFromPointToSegment(
	const double Px, const double Py, const double x1, const double y1,
	const double x2, const double y2, T& out_x, T& out_y)
{
	double ox, oy;
	closestFromPointToSegment(Px, Py, x1, y1, x2, y2, ox, oy);
	out_x = static_cast<T>(ox);
	out_y = static_cast<T>(oy);
	return distanceBetweenPoints(Px, Py, ox, oy);
}

/** Returns the intersection point, and if it exists, between two segments.
 */
bool SegmentsIntersection(
	const double x1, const double y1, const double x2, const double y2,
	const double x3, const double y3, const double x4, const double y4,
	double& ix, double& iy);

/** Returns the intersection point, and if it exists, between two segments.
 */
bool SegmentsIntersection(
	const double x1, const double y1, const double x2, const double y2,
	const double x3, const double y3, const double x4, const double y4,
	float& ix, float& iy);

/** Returns true if the 2D point (px,py) falls INTO the given polygon.
 * \sa pointIntoQuadrangle
 */
bool pointIntoPolygon2D(
	double px, double py, unsigned int polyEdges, const double* poly_xs,
	const double* poly_ys);

/** Specialized method to check whether a point (x,y) falls into a quadrangle.
 * \sa pointIntoPolygon2D
 */
template <typename T>
bool pointIntoQuadrangle(
	T x, T y, T v1x, T v1y, T v2x, T v2y, T v3x, T v3y, T v4x, T v4y)
{
	using mrpt::sign;

	const T a1 = atan2(v1y - y, v1x - x);
	const T a2 = atan2(v2y - y, v2x - x);
	const T a3 = atan2(v3y - y, v3x - x);
	const T a4 = atan2(v4y - y, v4x - x);

	// The point is INSIDE iff all the signs of the angles between each vertex
	//  and the next one are equal.
	const T da1 = mrpt::math::wrapToPi(a2 - a1);
	const T da2 = mrpt::math::wrapToPi(a3 - a2);
	if (sign(da1) != sign(da2)) return false;

	const T da3 = mrpt::math::wrapToPi(a4 - a3);
	if (sign(da2) != sign(da3)) return false;

	const T da4 = mrpt::math::wrapToPi(a1 - a4);
	return (sign(da3) == sign(da4) && (sign(da4) == sign(da1)));
}

/** Returns the closest distance of a given 2D point to a polygon, or "0" if the
 * point is INTO the polygon or its perimeter.
 */
double distancePointToPolygon2D(
	double px, double py, unsigned int polyEdges, const double* poly_xs,
	const double* poly_ys);

/** Calculates the minimum distance between a pair of lines.
  The lines are given by:
	- Line 1 = P1 + f (P2-P1)
	- Line 2 = P3 + f (P4-P3)
  The Euclidean distance is returned in "dist", and the mid point between the
  lines in (x,y,z)
  \return It returns false if there is no solution, i.e. lines are (almost, up
  to EPS) parallel.
 */
bool minDistBetweenLines(
	double p1_x, double p1_y, double p1_z, double p2_x, double p2_y,
	double p2_z, double p3_x, double p3_y, double p3_z, double p4_x,
	double p4_y, double p4_z, double& x, double& y, double& z, double& dist);

/** Returns whether two rotated rectangles intersect.
 *  The first rectangle is not rotated and given by
 * (R1_x_min,R1_x_max)-(R1_y_min,R1_y_max).
 *  The second rectangle is given is a similar way, but it is internally rotated
 * according
 *   to the given coordinates translation
 * (R2_pose_x,R2_pose_y,R2_pose_phi(radians)), relative
 *   to the coordinates system of rectangle 1.
 */
bool RectanglesIntersection(
	double R1_x_min, double R1_x_max, double R1_y_min, double R1_y_max,
	double R2_x_min, double R2_x_max, double R2_y_min, double R2_y_max,
	double R2_pose_x, double R2_pose_y, double R2_pose_phi);

/** Computes an axis base (a set of three 3D normal vectors) with the given
  vector being the first of them ("X")
  * NOTE: Make sure of passing all floats or doubles and that the template of
  the receiving matrix is of the same type!
  *
  *  If   \f$ d = [ dx ~ dy ~ dz ] \f$ is the input vector, then this function
  returns a matrix \f$ M \f$ such as:
  *
	\f[  M = \left(
			\begin{array}{c c c}
			v^1_x ~ v^2_x ~ v^3_x \\
			v^1_y ~ v^2_y ~ v^3_y \\
			v^1_z ~ v^2_z ~ v^3_z
			\end{array} \right)
	\f]
  *
  *   And the three normal vectors are computed as:
	*
	*  \f[ v^1 = \frac{d}{|d|}  \f]
	*
	* If (dx!=0 or dy!=0):
	* \f[ v^2 = \frac{[-dy ~ dx ~ 0 ]}{\sqrt{dx^2+dy^2}}  \f]
	* otherwise (the direction vector is vertical):
	* \f[ v^2 = [1 ~ 0 ~ 0]  \f]
	*
	* And finally, the third vector is the cross product of the others:
	*
	*    \f[ v^3 = v^1 \times v^2  \f]
  *
  * \return The 3x3 matrix (CMatrixDynamic<T>), containing one vector
  per column.
  * \except Throws an std::exception on invalid input (i.e. null direction
  vector)
	* \sa generateAxisBaseFromDirectionAndAxis()
  *
  * (JLB @ 18-SEP-2007)
  */
CMatrixDouble33 generateAxisBaseFromDirection(double dx, double dy, double dz);

/** @} */  // end of misc. geom. methods

/** @} */  // end of grouping

}  // namespace mrpt::math
