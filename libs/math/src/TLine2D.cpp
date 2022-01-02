/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "math-precomp.h"  // Precompiled headers
//
#include <mrpt/math/TLine2D.h>
#include <mrpt/math/TLine3D.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TSegment2D.h>
#include <mrpt/math/epsilon.h>
#include <mrpt/serialization/CArchive.h>  // impl of << operator

#include <cmath>
#include <iostream>

using namespace mrpt::math;

static_assert(std::is_trivially_copyable_v<TLine2D>);

TLine2D TLine2D::FromCoefficientsABC(double A, double B, double C)
{
	return TLine2D(A, B, C);
}

TLine2D TLine2D::FromTwoPoints(const TPoint2D& p1, const TPoint2D& p2)
{
	return TLine2D(p1, p2);
}

double TLine2D::evaluatePoint(const TPoint2D& point) const
{
	return coefs[0] * point.x + coefs[1] * point.y + coefs[2];
}
bool TLine2D::contains(const TPoint2D& point) const
{
	return std::abs(distance(point)) < getEpsilon();
}
double TLine2D::distance(const TPoint2D& point) const
{
	return std::abs(evaluatePoint(point)) /
		sqrt(coefs[0] * coefs[0] + coefs[1] * coefs[1]);
}
double TLine2D::signedDistance(const TPoint2D& point) const
{
	return evaluatePoint(point) /
		sqrt(coefs[0] * coefs[0] + coefs[1] * coefs[1]);
}
void TLine2D::getNormalVector(double (&vector)[2]) const
{
	vector[0] = coefs[0];
	vector[1] = coefs[1];
}
void TLine2D::unitarize()
{
	double s = sqrt(coefs[0] * coefs[0] + coefs[1] * coefs[1]);
	for (double& coef : coefs)
		coef /= s;
}
void TLine2D::getDirectorVector(double (&vector)[2]) const
{
	vector[0] = -coefs[1];
	vector[1] = coefs[0];
}
void TLine2D::generate3DObject(TLine3D& l) const { l = TLine3D(*this); }
void TLine2D::getAsPose2D(TPose2D& outPose) const
{
	// Line's director vector is (-coefs[1],coefs[0]).
	// If line is horizontal, force x=0. Else, force y=0. In both cases, we'll
	// find a suitable point.
	outPose.phi = atan2(coefs[0], -coefs[1]);
	if (std::abs(coefs[0]) < getEpsilon())
	{
		outPose.x = 0;
		outPose.y = -coefs[2] / coefs[1];
	}
	else
	{
		outPose.x = -coefs[2] / coefs[0];
		outPose.y = 0;
	}
}
void TLine2D::getAsPose2DForcingOrigin(
	const TPoint2D& origin, TPose2D& outPose) const
{
	if (!contains(origin))
		throw std::logic_error("Base point is not contained in the line");
	outPose = origin;
	// Line's director vector is (-coefs[1],coefs[0]).
	outPose.phi = atan2(coefs[0], -coefs[1]);
}
TLine2D::TLine2D(const TPoint2D& p1, const TPoint2D& p2)
{
	if (p1 == p2) throw std::logic_error("Both points are the same");
	coefs[0] = p2.y - p1.y;
	coefs[1] = p1.x - p2.x;
	coefs[2] = p2.x * p1.y - p2.y * p1.x;
}
TLine2D::TLine2D(const TSegment2D& s)
{
	coefs[0] = s.point2.y - s.point1.y;
	coefs[1] = s.point1.x - s.point2.x;
	coefs[2] = s.point2.x * s.point1.y - s.point2.y * s.point1.x;
	// unitarize();	//¿?
}
TLine2D::TLine2D(const TLine3D& l)
{
	// Line's projection to Z plane may be a point.
	if (hypot(l.director[0], l.director[1]) < getEpsilon())
		throw std::logic_error("Line is normal to projection plane");
	coefs[0] = -l.director[1];
	coefs[1] = l.director[0];
	coefs[2] = l.pBase.x * l.director[1] - l.pBase.y * l.director[0];
}

std::string TLine2D::asString() const
{
	return mrpt::format(
		"[%10.05f, %10.05f, %10.05f]", coefs[0], coefs[1], coefs[2]);
}

mrpt::serialization::CArchive& mrpt::math::operator>>(
	mrpt::serialization::CArchive& in, mrpt::math::TLine2D& l)
{
	return in >> l.coefs[0] >> l.coefs[1] >> l.coefs[2];
}
mrpt::serialization::CArchive& mrpt::math::operator<<(
	mrpt::serialization::CArchive& out, const mrpt::math::TLine2D& l)
{
	return out << l.coefs[0] << l.coefs[1] << l.coefs[2];
}

std::ostream& mrpt::math::operator<<(std::ostream& o, const TLine2D& p)
{
	o << p.asString();
	return o;
}
