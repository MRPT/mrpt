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
#include <mrpt/math/TLine2D.h>
#include <mrpt/math/TLine3D.h>
#include <mrpt/math/epsilon.h>
#include <mrpt/math/geometry.h>	 // distance()
#include <mrpt/math/ops_containers.h>  // squareNorm()
#include <mrpt/serialization/CArchive.h>  // impl of << operator

using namespace mrpt::math;

static_assert(std::is_trivially_copyable_v<TLine3D>);

void TLine3D::generate2DObject(TLine2D& l) const { l = TLine2D(*this); }

TLine3D TLine3D::FromPointAndDirector(
	const TPoint3D& basePoint, const TVector3D& directorVector)
{
	TLine3D l;
	l.pBase = basePoint;
	l.director = directorVector;
	return l;
}

TLine3D TLine3D::FromTwoPoints(const TPoint3D& p1, const TPoint3D& p2)
{
	return TLine3D(p1, p2);
}

bool TLine3D::contains(const TPoint3D& point) const
{
	double dx = point.x - pBase.x;
	double dy = point.y - pBase.y;
	double dz = point.z - pBase.z;
	if (std::abs(dx) < getEpsilon() && std::abs(dy) < getEpsilon() &&
		std::abs(dz) < getEpsilon())
		return true;
	//       dx          dy          dz
	// if -----------=-----------=-----------, point is inside the line.
	//   director[0] director[1] director[2]
	return (std::abs(dx * director[1] - dy * director[0]) < getEpsilon()) &&
		(std::abs(dx * director[2] - dz * director[0]) < getEpsilon()) &&
		(std::abs(dy * director[2] - dz * director[1]) < getEpsilon());
}
double TLine3D::distance(const TPoint3D& point) const
{
	// Let d be line's base point minus the argument. Then,
	// d·director/(|d|·|director|) equals both vector's cosine.
	// So, d·director/|director| equals d's projection over director. Then,
	// distance is sqrt(|d|²-(d·director/|director|)²).
	double d[3] = {point.x - pBase.x, point.y - pBase.y, point.z - pBase.z};
	double dv = 0, d2 = 0, v2 = 0;
	for (size_t i = 0; i < 3; i++)
	{
		dv += d[i] * director[i];
		d2 += d[i] * d[i];
		v2 += director[i] * director[i];
	}
	return sqrt(d2 - (dv * dv) / v2);
}
void TLine3D::unitarize()
{
	const double norm = director.norm();
	ASSERT_(norm > 0);
	director *= 1.0 / norm;
}
TLine3D::TLine3D(const TPoint3D& p1, const TPoint3D& p2)
{
	if (std::abs(math::distance(p1, p2)) < getEpsilon())
		throw std::logic_error("Both points are the same");
	pBase = p1;
	director[0] = p2.x - p1.x;
	director[1] = p2.y - p1.y;
	director[2] = p2.z - p1.z;
}
TLine3D::TLine3D(const TSegment3D& s)
{
	pBase = s.point1;
	director[0] = s.point2.x - s.point1.x;
	director[1] = s.point2.y - s.point1.y;
	director[2] = s.point2.z - s.point1.z;
}
TLine3D::TLine3D(const TLine2D& l)
{
	director[0] = -l.coefs[1];
	director[1] = l.coefs[0];
	director[2] = 0;
	// We assume that either l.coefs[0] or l.coefs[1] is not null. Respectively,
	// either y or x can be used as free cordinate.
	if (std::abs(l.coefs[0]) >= getEpsilon())
	{
		pBase.x = -l.coefs[2] / l.coefs[0];
		pBase.y = 0;
	}
	else
	{
		pBase.x = 0;
		pBase.y = -l.coefs[1] / l.coefs[0];
	}
	pBase.z = 0;
}

std::string TLine3D::asString() const
{
	return mrpt::format(
		"P=[%10.05f, %10.05f, %10.05f] u=[%10.05f, %10.05f, %10.05f]", pBase.x,
		pBase.y, pBase.z, director.x, director.y, director.z);
}

mrpt::serialization::CArchive& mrpt::math::operator>>(
	mrpt::serialization::CArchive& in, mrpt::math::TLine3D& l)
{
	return in >> l.pBase >> l.director[0] >> l.director[1] >> l.director[2];
}
mrpt::serialization::CArchive& mrpt::math::operator<<(
	mrpt::serialization::CArchive& out, const mrpt::math::TLine3D& l)
{
	return out << l.pBase << l.director[0] << l.director[1] << l.director[2];
}

std::ostream& mrpt::math::operator<<(std::ostream& o, const TLine3D& p)
{
	o << p.asString();
	return o;
}
