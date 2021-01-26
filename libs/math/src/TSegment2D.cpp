/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "math-precomp.h"  // Precompiled headers
//
#include <mrpt/math/TSegment2D.h>
#include <mrpt/math/geometry.h>	 // distance()
#include <mrpt/serialization/CArchive.h>  // impl of << operator

using namespace mrpt::math;

double TSegment2D::length() const { return math::distance(point1, point2); }
double TSegment2D::distance(const TPoint2D& point) const
{
	return std::abs(signedDistance(point));
}
double TSegment2D::signedDistance(const TPoint2D& point) const
{
	// It is reckoned whether the perpendicular line to the TSegment2D which
	// passes through point crosses or not the referred segment,
	// or what is the same, whether point makes an obtuse triangle with the
	// segment or not (being the longest segment one between the point and
	// either end of TSegment2D).
	const double d1 = math::distance(point, point1);
	if (point1 == point2) return d1;

	const double d2 = math::distance(point, point2);
	const double d3 = length();
	const double ds1 = square(d1);
	const double ds2 = square(d2);
	const double ds3 = square(d3);
	if (ds1 > (ds2 + ds3) || ds2 > (ds1 + ds3))
		// Fix sign:
		return std::min(d1, d2) *
			(TLine2D(*this).signedDistance(point) < 0 ? -1 : 1);
	else
		return TLine2D(*this).signedDistance(point);
}
bool TSegment2D::contains(const TPoint2D& point) const
{
	return std::abs(
			   math::distance(point1, point) + math::distance(point2, point) -
			   math::distance(point1, point2)) < getEpsilon();
}
void TSegment2D::generate3DObject(TSegment3D& s) const
{
	s = TSegment3D(*this);
}
TSegment2D::TSegment2D(const TSegment3D& s)
{
	point1 = TPoint2D(s.point1);
	point2 = TPoint2D(s.point2);
	if (point1 == point2)
		throw std::logic_error("Segment is normal to projection plane");
}

bool TSegment2D::operator<(const TSegment2D& s) const
{
	if (point1 < s.point1) return true;
	else if (s.point1 < point1)
		return false;
	else
		return point2 < s.point2;
}

mrpt::serialization::CArchive& mrpt::math::operator>>(
	mrpt::serialization::CArchive& in, mrpt::math::TSegment2D& s)
{
	return in >> s.point1 >> s.point2;
}
mrpt::serialization::CArchive& mrpt::math::operator<<(
	mrpt::serialization::CArchive& out, const mrpt::math::TSegment2D& s)
{
	return out << s.point1 << s.point2;
}
