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
#include <mrpt/core/bits_math.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TTwist3D.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt::math;
using mrpt::DEG2RAD;
using mrpt::RAD2DEG;

static_assert(std::is_trivially_copyable_v<TTwist3D>);

void TTwist3D::asString(std::string& s) const
{
	s = mrpt::format(
		"[%f %f %f  %f %f %f]", vx, vy, vz, RAD2DEG(wx), RAD2DEG(wy),
		RAD2DEG(wz));
}
void TTwist3D::fromString(const std::string& s)
{
	CMatrixDouble m;
	if (!m.fromMatlabStringFormat(s))
		THROW_EXCEPTION_FMT(
			"Malformed expression in ::fromString, s=\"%s\"", s.c_str());
	ASSERTMSG_(
		m.rows() == 1 && m.cols() == 6, "Wrong size of vector in ::fromString");
	for (int i = 0; i < 3; i++)
		(*this)[i] = m(0, i);
	for (int i = 0; i < 3; i++)
		(*this)[3 + i] = DEG2RAD(m(0, 3 + i));
}
// Transform all 6 components for a change of reference frame from "A" to
// another frame "B" whose rotation with respect to "A" is given by `rot`. The
// translational part of the pose is ignored
void TTwist3D::rotate(const TPose3D& rot)
{
	const TTwist3D t = *this;
	mrpt::math::CMatrixDouble33 R;
	rot.getRotationMatrix(R);
	vx = R(0, 0) * t.vx + R(0, 1) * t.vy + R(0, 2) * t.vz;
	vy = R(1, 0) * t.vx + R(1, 1) * t.vy + R(1, 2) * t.vz;
	vz = R(2, 0) * t.vx + R(2, 1) * t.vy + R(2, 2) * t.vz;

	wx = R(0, 0) * t.wx + R(0, 1) * t.wy + R(0, 2) * t.wz;
	wy = R(1, 0) * t.wx + R(1, 1) * t.wy + R(1, 2) * t.wz;
	wz = R(2, 0) * t.wx + R(2, 1) * t.wy + R(2, 2) * t.wz;
}
bool TTwist3D::operator==(const TTwist3D& o) const
{
	return vx == o.vx && vy == o.vy && vz == o.vz && wx == o.wx && wy == o.wy &&
		wz == o.wz;
}
bool TTwist3D::operator!=(const TTwist3D& o) const { return !(*this == o); }

mrpt::serialization::CArchive& mrpt::math::operator>>(
	mrpt::serialization::CArchive& in, mrpt::math::TTwist3D& o)
{
	for (size_t i = 0; i < o.size(); i++)
		in >> o[i];
	return in;
}
mrpt::serialization::CArchive& mrpt::math::operator<<(
	mrpt::serialization::CArchive& out, const mrpt::math::TTwist3D& o)
{
	for (size_t i = 0; i < o.size(); i++)
		out << o[i];
	return out;
}
