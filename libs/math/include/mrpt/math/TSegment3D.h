/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPoseOrPoint.h>
#include <mrpt/math/TSegment2D.h>

namespace mrpt::math
{
/** 3D segment, consisting of two points.
 * \sa TSegment2D,TLine3D,TPlane,TPolygon3D,TPoint3D
 */
struct TSegment3D
{
   public:
	TPoint3D point1;  //!< origin point
	TPoint3D point2;  //!< final point

	/** Segment length */
	double length() const;

	/** Distance to point */
	double distance(const TPoint3D& point) const;

	/** Distance to another segment */
	double distance(const TSegment3D& segment) const;
	/**
	 * Check whether a point is inside the segment.
	 */
	bool contains(const TPoint3D& point) const;
	/** Access to points using operator[0-1] */
	TPoint3D& operator[](size_t i)
	{
		switch (i)
		{
			case 0:
				return point1;
			case 1:
				return point2;
			default:
				throw std::out_of_range("index out of range");
		}
	}
	/** Access to points using operator[0-1] */
	const TPoint3D& operator[](size_t i) const
	{
		switch (i)
		{
			case 0:
				return point1;
			case 1:
				return point2;
			default:
				throw std::out_of_range("index out of range");
		}
	}
	/**
	 * Projection into 2D space, discarding the z.
	 */
	void generate2DObject(TSegment2D& s) const;
	/**
	 * Segment's central point.
	 */
	void getCenter(TPoint3D& p) const
	{
		p.x = (point1.x + point2.x) / 2;
		p.y = (point1.y + point2.y) / 2;
		p.z = (point1.z + point2.z) / 2;
	}
	/**
	 * Constructor from both points.
	 */
	TSegment3D(const TPoint3D& p1, const TPoint3D& p2) : point1(p1), point2(p2)
	{
	}
	/**
	 * Fast default constructor. Initializes to garbage.
	 */
	TSegment3D() = default;
	/**
	 * Constructor from 2D object. Sets the z to zero.
	 */
	explicit TSegment3D(const TSegment2D& s)
		: point1(s.point1), point2(s.point2)
	{
	}

	bool operator<(const TSegment3D& s) const;
};

inline bool operator==(const TSegment3D& s1, const TSegment3D& s2)
{
	return (s1.point1 == s2.point1) && (s1.point2 == s2.point2);
}

inline bool operator!=(const TSegment3D& s1, const TSegment3D& s2)
{
	return (s1.point1 != s2.point1) || (s1.point2 != s2.point2);
}

mrpt::serialization::CArchive& operator>>(
	mrpt::serialization::CArchive& in, mrpt::math::TSegment3D& s);
mrpt::serialization::CArchive& operator<<(
	mrpt::serialization::CArchive& out, const mrpt::math::TSegment3D& s);

}  // namespace mrpt::math

namespace mrpt::typemeta
{
// Specialization must occur in the same namespace
MRPT_DECLARE_TTYPENAME_NO_NAMESPACE(TSegment3D, mrpt::math)
}  // namespace mrpt::typemeta
