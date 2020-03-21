/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "math-precomp.h"  // Precompiled headers

#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>

using namespace mrpt::math;

static_assert(std::is_trivially_copyable_v<TPose2D>);

TPose2D::TPose2D(const TPoint2D& p) : x(p.x), y(p.y), phi(0.0) {}
TPose2D::TPose2D(const TPoint3D& p) : x(p.x), y(p.y), phi(0.0) {}
TPose2D::TPose2D(const TPose3D& p) : x(p.x), y(p.y), phi(p.yaw) {}
void TPose2D::asString(std::string& s) const
{
	s = mrpt::format("[%f %f %f]", x, y, RAD2DEG(phi));
}
void TPose2D::fromString(const std::string& s)
{
	CMatrixDouble m;
	if (!m.fromMatlabStringFormat(s))
		THROW_EXCEPTION("Malformed expression in ::fromString");
	ASSERTMSG_(
		m.rows() == 1 && m.cols() == 3, "Wrong size of vector in ::fromString");
	x = m(0, 0);
	y = m(0, 1);
	phi = DEG2RAD(m(0, 2));
}
mrpt::math::TPose2D mrpt::math::TPose2D::operator+(
	const mrpt::math::TPose2D& b) const
{
	const double A_cosphi = cos(this->phi), A_sinphi = sin(this->phi);
	// Use temporary variables for the cases (A==this) or (B==this)
	const double new_x = this->x + b.x * A_cosphi - b.y * A_sinphi;
	const double new_y = this->y + b.x * A_sinphi + b.y * A_cosphi;
	const double new_phi = mrpt::math::wrapToPi(this->phi + b.phi);

	return mrpt::math::TPose2D(new_x, new_y, new_phi);
}

mrpt::math::TPose2D mrpt::math::TPose2D::operator-(
	const mrpt::math::TPose2D& b) const
{
	const double B_cosphi = cos(b.phi), B_sinphi = sin(b.phi);

	const double new_x =
		(this->x - b.x) * B_cosphi + (this->y - b.y) * B_sinphi;
	const double new_y =
		-(this->x - b.x) * B_sinphi + (this->y - b.y) * B_cosphi;
	const double new_phi = mrpt::math::wrapToPi(this->phi - b.phi);

	return mrpt::math::TPose2D(new_x, new_y, new_phi);
}
TPoint2D TPose2D::composePoint(const TPoint2D l) const
{
	const double ccos = ::cos(phi), csin = ::sin(phi);
	return {x + l.x * ccos - l.y * csin, y + l.x * csin + l.y * ccos};
}
mrpt::math::TPoint2D TPose2D::operator+(const mrpt::math::TPoint2D& b) const
{
	return this->composePoint(b);
}

mrpt::math::TPoint2D TPose2D::inverseComposePoint(const TPoint2D g) const
{
	const double Ax = g.x - x, Ay = g.y - y, ccos = ::cos(phi),
				 csin = ::sin(phi);
	return {Ax * ccos + Ay * csin, -Ax * csin + Ay * ccos};
}
