/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "math-precomp.h"  // Precompiled headers

#include <mrpt/math/TLine3D.h>
#include <mrpt/math/TPlane.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/epsilon.h>
#include <mrpt/math/geometry.h>  // getAngle()
#include <mrpt/math/ops_containers.h>  // dotProduct()
#include <mrpt/serialization/CArchive.h>  // impl of << operator

using namespace mrpt::math;

static_assert(std::is_trivially_copyable_v<TPlane>);

double TPlane::evaluatePoint(const TPoint3D& point) const
{
	return dotProduct<3, double>(coefs, point) + coefs[3];
}
bool TPlane::contains(const TPoint3D& point) const
{
	return distance(point) < getEpsilon();
}
bool TPlane::contains(const TLine3D& line) const
{
	if (!contains(line.pBase)) return false;  // Base point must be contained
	return std::abs(getAngle(*this, line)) <
		   getEpsilon();  // Plane's normal must be normal to director vector
}
double TPlane::distance(const TPoint3D& point) const
{
	return std::abs(evaluatePoint(point)) / sqrt(squareNorm<3, double>(coefs));
}
double TPlane::distance(const TLine3D& line) const
{
	if (std::abs(getAngle(*this, line)) >= getEpsilon())
		return 0;  // Plane crosses with line
	else
		return distance(line.pBase);
}
TVector3D TPlane::getNormalVector() const
{
	TVector3D v;
	for (int i = 0; i < 3; i++) v[i] = coefs[i];
	return v;
}

TVector3D TPlane::getUnitaryNormalVector() const
{
	TVector3D vec;
	const double s = sqrt(squareNorm<3, double>(coefs));
	ASSERT_ABOVE_(s, getEpsilon());
	const double k = 1.0 / s;
	for (int i = 0; i < 3; i++) vec[i] = coefs[i] * k;
	return vec;
}

void TPlane::unitarize()
{
	double s = sqrt(squareNorm<3, double>(coefs));
	for (double& coef : coefs) coef /= s;
}

// Returns a 6D pose such as its XY plane coincides with the plane
void TPlane::getAsPose3D(mrpt::math::TPose3D& outPose) const
{
	const TVector3D normal = getUnitaryNormalVector();
	CMatrixDouble44 AXIS = generateAxisBaseFromDirectionAndAxis(normal, 2);
	for (size_t i = 0; i < 3; i++)
		if (std::abs(coefs[i]) >= getEpsilon())
		{
			AXIS(i, 3) = -coefs[3] / coefs[i];
			break;
		}
	outPose.fromHomogeneousMatrix(AXIS);
}
void TPlane::getAsPose3DForcingOrigin(
	const TPoint3D& center, TPose3D& pose) const
{
	if (!contains(center))
		throw std::logic_error("Base point is not in the plane.");
	const TVector3D normal = getUnitaryNormalVector();
	CMatrixDouble44 AXIS = generateAxisBaseFromDirectionAndAxis(normal, 2);
	for (size_t i = 0; i < 3; i++) AXIS(i, 3) = center[i];
	pose.fromHomogeneousMatrix(AXIS);
}
TPlane::TPlane(const TPoint3D& p1, const TPoint3D& p2, const TPoint3D& p3)
{
	double dx1 = p2.x - p1.x;
	double dy1 = p2.y - p1.y;
	double dz1 = p2.z - p1.z;
	double dx2 = p3.x - p1.x;
	double dy2 = p3.y - p1.y;
	double dz2 = p3.z - p1.z;
	coefs[0] = dy1 * dz2 - dy2 * dz1;
	coefs[1] = dz1 * dx2 - dz2 * dx1;
	coefs[2] = dx1 * dy2 - dx2 * dy1;
	if (std::abs(coefs[0]) < getEpsilon() &&
		std::abs(coefs[1]) < getEpsilon() && std::abs(coefs[2]) < getEpsilon())
		throw std::logic_error("Points are linearly dependent");
	coefs[3] = -coefs[0] * p1.x - coefs[1] * p1.y - coefs[2] * p1.z;
}
TPlane::TPlane(const TPoint3D& p1, const TLine3D& r2)
{
	double dx1 = p1.x - r2.pBase.x;
	double dy1 = p1.y - r2.pBase.y;
	double dz1 = p1.z - r2.pBase.z;
	coefs[0] = dy1 * r2.director[2] - dz1 * r2.director[1];
	coefs[1] = dz1 * r2.director[0] - dx1 * r2.director[2];
	coefs[2] = dx1 * r2.director[1] - dy1 * r2.director[0];
	if (std::abs(coefs[0]) < getEpsilon() &&
		std::abs(coefs[1]) < getEpsilon() && std::abs(coefs[2]) < getEpsilon())
		throw std::logic_error("Point is contained in the line");
	coefs[3] = -coefs[0] * p1.x - coefs[1] * p1.y - coefs[2] * p1.z;
}
TPlane::TPlane(const TPoint3D& p1, const TVector3D& normal)
{
	const double normal_norm = normal.norm();
	ASSERT_ABOVE_(normal_norm, getEpsilon());

	// Ensure we have a unit vector:
	const auto n = normal * (1. / normal_norm);
	coefs[0] = n.x;
	coefs[1] = n.y;
	coefs[2] = n.z;
	coefs[3] = -coefs[0] * p1.x - coefs[1] * p1.y - coefs[2] * p1.z;
}
TPlane::TPlane(const TLine3D& r1, const TLine3D& r2)
{
	crossProduct3D(r1.director, r2.director, coefs);
	coefs[3] =
		-coefs[0] * r1.pBase.x - coefs[1] * r1.pBase.y - coefs[2] * r1.pBase.z;
	if (std::abs(coefs[0]) < getEpsilon() &&
		std::abs(coefs[1]) < getEpsilon() && std::abs(coefs[2]) < getEpsilon())
	{
		// Lines are parallel
		if (r1.contains(r2.pBase)) throw std::logic_error("Lines are the same");
		// Use a line's director vector and both pBase's difference to create
		// the plane.
		double d[3];
		for (size_t i = 0; i < 3; i++) d[i] = r1.pBase[i] - r2.pBase[i];
		crossProduct3D(r1.director, d, coefs);
		coefs[3] = -coefs[0] * r1.pBase.x - coefs[1] * r1.pBase.y -
				   coefs[2] * r1.pBase.z;
	}
	else if (std::abs(evaluatePoint(r2.pBase)) >= getEpsilon())
		throw std::logic_error("Lines do not intersect");
}

mrpt::serialization::CArchive& mrpt::math::operator>>(
	mrpt::serialization::CArchive& in, mrpt::math::TPlane& p)
{
	return in >> p.coefs[0] >> p.coefs[1] >> p.coefs[2] >> p.coefs[3];
}
mrpt::serialization::CArchive& mrpt::math::operator<<(
	mrpt::serialization::CArchive& out, const mrpt::math::TPlane& p)
{
	return out << p.coefs[0] << p.coefs[1] << p.coefs[2] << p.coefs[3];
}
