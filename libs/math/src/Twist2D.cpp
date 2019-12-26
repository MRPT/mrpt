/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "math-precomp.h"  // Precompiled headers

#include <mrpt/core/bits_math.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TTwist2D.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt::math;
using mrpt::DEG2RAD;
using mrpt::RAD2DEG;

static_assert(std::is_trivially_copyable_v<TTwist2D>);

void TTwist2D::asString(std::string& s) const
{
	s = mrpt::format("[%f %f %f]", vx, vy, RAD2DEG(omega));
}
void TTwist2D::fromString(const std::string& s)
{
	CMatrixDouble m;
	if (!m.fromMatlabStringFormat(s))
		THROW_EXCEPTION("Malformed expression in ::fromString");
	ASSERTMSG_(
		m.rows() == 1 && m.cols() == 3, "Wrong size of vector in ::fromString");
	vx = m(0, 0);
	vy = m(0, 1);
	omega = DEG2RAD(m(0, 2));
}
// Transform the (vx,vy) components for a counterclockwise rotation of `ang`
// radians
void TTwist2D::rotate(const double ang)
{
	const double nvx = vx * cos(ang) - vy * sin(ang);
	const double nvy = vx * sin(ang) + vy * cos(ang);
	vx = nvx;
	vy = nvy;
}
bool TTwist2D::operator==(const TTwist2D& o) const
{
	return vx == o.vx && vy == o.vy && omega == o.omega;
}
bool TTwist2D::operator!=(const TTwist2D& o) const { return !(*this == o); }
mrpt::math::TPose2D TTwist2D::operator*(const double dt) const
{
	return mrpt::math::TPose2D(vx * dt, vy * dt, omega * dt);
}

mrpt::serialization::CArchive& mrpt::math::operator>>(
	mrpt::serialization::CArchive& in, mrpt::math::TTwist2D& o)
{
	for (size_t i = 0; i < o.size(); i++) in >> o[i];
	return in;
}
mrpt::serialization::CArchive& mrpt::math::operator<<(
	mrpt::serialization::CArchive& out, const mrpt::math::TTwist2D& o)
{
	for (size_t i = 0; i < o.size(); i++) out << o[i];
	return out;
}
