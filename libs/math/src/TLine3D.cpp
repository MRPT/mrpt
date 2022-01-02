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
#include <mrpt/math/TObject3D.h>
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

std::optional<double> TLine3D::distance(
	const TLine3D& L2, const mrpt::optional_ref<TPoint3D>& outMidPoint) const
{
	/*
	The lines are given by:
		- Line 1 = P1 + f (P2-P1)
		- Line 2 = P3 + f (P4-P3)
	The Euclidean distance is returned in "dist", and the mid point between the
	  lines in (x,y,z)
	  */

	const double EPS = 1e-20;
	const double p1_x = pBase.x, p1_y = pBase.y, p1_z = pBase.z;
	const double p3_x = L2.pBase.x, p3_y = L2.pBase.y, p3_z = L2.pBase.z;

	double p13_x, p13_y, p13_z;

	double d1343, d4321, d1321, d4343, d2121;
	double numer, denom;

	p13_x = p1_x - p3_x;
	p13_y = p1_y - p3_y;
	p13_z = p1_z - p3_z;

	const double p43_x = L2.director.x, p43_y = L2.director.y,
				 p43_z = L2.director.z;

	if (fabs(p43_x) < EPS && fabs(p43_y) < EPS && fabs(p43_z) < EPS)
		THROW_EXCEPTION("L2 director vector norm is < EPS");

	const double p21_x = director.x, p21_y = director.y, p21_z = director.z;

	if (fabs(p21_x) < EPS && fabs(p21_y) < EPS && fabs(p21_z) < EPS)
		THROW_EXCEPTION("thid line director vector norm is < EPS");

	d1343 = p13_x * p43_x + p13_y * p43_y + p13_z * p43_z;
	d4321 = p43_x * p21_x + p43_y * p21_y + p43_z * p21_z;
	d1321 = p13_x * p21_x + p13_y * p21_y + p13_z * p21_z;
	d4343 = p43_x * p43_x + p43_y * p43_y + p43_z * p43_z;
	d2121 = p21_x * p21_x + p21_y * p21_y + p21_z * p21_z;

	denom = d2121 * d4343 - d4321 * d4321;
	if (fabs(denom) < EPS) return {};

	numer = d1343 * d4321 - d1321 * d4343;

	double mua = numer / denom;
	double mub = (d1343 + d4321 * mua) / d4343;
	double pa_x, pa_y, pa_z;
	double pb_x, pb_y, pb_z;

	pa_x = p1_x + mua * p21_x;
	pa_y = p1_y + mua * p21_y;
	pa_z = p1_z + mua * p21_z;

	pb_x = p3_x + mub * p43_x;
	pb_y = p3_y + mub * p43_y;
	pb_z = p3_z + mub * p43_z;

	const double dist = std::sqrt(
		square(pa_x - pb_x) + square(pa_y - pb_y) + square(pa_z - pb_z));

	// the mid point:
	if (outMidPoint)
	{
		outMidPoint.value().get() = TPoint3D(
			0.5 * (pa_x + pb_x), 0.5 * (pa_y + pb_y), 0.5 * (pa_z + pb_z));
	}

	return {dist};
}

TPoint3D TLine3D::closestPointTo(const TPoint3D& p) const
{
	const auto pl = mrpt::math::TPlane::FromPointAndNormal(p, director);
	TObject3D inter;
	bool ok = mrpt::math::intersect(pl, *this, inter);
	ASSERT_(ok);
	if (inter.isLine()) return p;

	ASSERT_(inter.isPoint());
	return inter.getAs<TPoint3D>();
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
