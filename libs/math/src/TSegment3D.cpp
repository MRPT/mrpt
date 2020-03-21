/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "math-precomp.h"  // Precompiled headers

#include <mrpt/math/TLine3D.h>
#include <mrpt/math/TSegment3D.h>
#include <mrpt/math/geometry.h>  // distance()
#include <mrpt/serialization/CArchive.h>  // impl of << operator
#include <Eigen/Dense>

using namespace mrpt::math;

void TSegment3D::generate2DObject(TSegment2D& s) const
{
	s = TSegment2D(*this);
}

double TSegment3D::length() const { return math::distance(point1, point2); }
double TSegment3D::distance(const TPoint3D& point) const
{
	return std::min(
		std::min(math::distance(point, point1), math::distance(point, point2)),
		TLine3D(*this).distance(point));
}
double TSegment3D::distance(const TSegment3D& segment) const
{
	Eigen::Vector3d u, v, w;
	TPoint3D diff_vect = point2 - point1;
	diff_vect.asVector(u);
	diff_vect = segment.point2 - segment.point1;
	diff_vect.asVector(v);
	diff_vect = point1 - segment.point1;
	diff_vect.asVector(w);
	double a = u.dot(u);  // always >= 0
	double b = u.dot(v);
	double c = v.dot(v);  // always >= 0
	double d = u.dot(w);
	double e = v.dot(w);
	double D = a * c - b * b;  // always >= 0
	double sc, sN, sD = D;  // sc = sN / sD, default sD = D >= 0
	double tc, tN, tD = D;  // tc = tN / tD, default tD = D >= 0

	// compute the line parameters of the two closest points
	if (D < 0.00000001)
	{  // the lines are almost parallel
		sN = 0.0;  // force using point P0 on segment S1
		sD = 1.0;  // to prevent possible division by 0.0 later
		tN = e;
		tD = c;
	}
	else
	{  // get the closest points on the infinite lines
		sN = (b * e - c * d);
		tN = (a * e - b * d);
		if (sN < 0.0)
		{  // sc < 0 => the s=0 edge is visible
			sN = 0.0;
			tN = e;
			tD = c;
		}
		else if (sN > sD)
		{  // sc > 1 => the s=1 edge is visible
			sN = sD;
			tN = e + b;
			tD = c;
		}
	}

	if (tN < 0.0)
	{  // tc < 0 => the t=0 edge is visible
		tN = 0.0;
		// recompute sc for this edge
		if (-d < 0.0)
			sN = 0.0;
		else if (-d > a)
			sN = sD;
		else
		{
			sN = -d;
			sD = a;
		}
	}
	else if (tN > tD)
	{  // tc > 1 => the t=1 edge is visible
		tN = tD;
		// recompute sc for this edge
		if ((-d + b) < 0.0)
			sN = 0;
		else if ((-d + b) > a)
			sN = sD;
		else
		{
			sN = (-d + b);
			sD = a;
		}
	}
	// finally do the division to get sc and tc
	sc = (fabs(sN) < 0.00000001 ? 0.0 : sN / sD);
	tc = (fabs(tN) < 0.00000001 ? 0.0 : tN / tD);

	// get the difference of the two closest points
	const auto dP = (w + (sc * u) - (tc * v)).eval();
	return dP.norm();  // return the closest distance
}
bool TSegment3D::contains(const TPoint3D& point) const
{
	// Not very intuitive, but very fast, method.
	return std::abs(
			   math::distance(point1, point) + math::distance(point2, point) -
			   math::distance(point1, point2)) < getEpsilon();
}

bool TSegment3D::operator<(const TSegment3D& s) const
{
	if (point1 < s.point1)
		return true;
	else if (s.point1 < point1)
		return false;
	else
		return point2 < s.point2;
}

mrpt::serialization::CArchive& mrpt::math::operator>>(
	mrpt::serialization::CArchive& in, mrpt::math::TSegment3D& s)
{
	return in >> s.point1 >> s.point2;
}
mrpt::serialization::CArchive& mrpt::math::operator<<(
	mrpt::serialization::CArchive& out, const mrpt::math::TSegment3D& s)
{
	return out << s.point1 << s.point2;
}
