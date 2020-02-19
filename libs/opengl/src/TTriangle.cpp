/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/opengl/TTriangle.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt::opengl;

// packet size= 3 vertices, each XYZ (float) + normal (XYZ float) + RGBA (u8)
static_assert(sizeof(TTriangle) == (sizeof(float) * (3 + 3) + 4) * 3);

void TTriangle::computeNormals()
{
	const float ax = x(1) - x(0);
	const float ay = y(1) - y(0);
	const float az = z(1) - z(0);
	const float bx = x(2) - x(0);
	const float by = y(2) - y(0);
	const float bz = z(2) - z(0);

	const mrpt::math::TVector3Df no = {ay * bz - az * by, -ax * bz + az * bx,
									   ax * by - ay * bx};
	for (auto& v : vertices) v.normal = no;
}

void TTriangle::writeTo(mrpt::serialization::CArchive& o) const
{
	for (const auto& p : vertices)
	{
		const auto& pp = p.position;
		o << pp.pt << pp.r << pp.g << pp.b << pp.a << p.normal;
	}
}
void TTriangle::readFrom(mrpt::serialization::CArchive& in)
{
	for (auto& p : vertices)
	{
		auto& pp = p.position;
		in >> pp.pt >> pp.r >> pp.g >> pp.b >> pp.a >> p.normal;
	}
}
