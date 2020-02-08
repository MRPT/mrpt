/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/core/exceptions.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPolygon3D.h>
#include <mrpt/serialization/serialization_frwds.h>

namespace mrpt::opengl
{
// Ensure 1-byte memory alignment, no additional stride bytes.
#pragma pack(push, 1)

/** A triangle with RGBA colors for each vertex (float scalars).
 * The structure is memory packed to 1-byte, to ensure it can be used in GPU
 * memory vertex arrays without unexpected paddings.
 *
 * \sa opengl::COpenGLScene, CSetOfTexturedTriangles
 * \ingroup mrpt_opengl_grp
 */
struct TTriangle
{
	TTriangle() = default;
	TTriangle(const mrpt::math::TPolygon3D& p)
	{
		ASSERT_EQUAL_(p.size(), 3U);
		for (size_t i = 0; i < 3; i++)
		{
			vertex[i].pt = p[i];
			vertex[i].R = vertex[i].G = vertex[i].B = 1;
		}
	}

	mrpt::math::TPointXYZRGBAf vertex[3];

	const float& x(int i) const { return vertex[i].pt.x; }
	const float& y(int i) const { return vertex[i].pt.y; }
	const float& z(int i) const { return vertex[i].pt.z; }
	const float& r(int i) const { return vertex[i].R; }
	const float& g(int i) const { return vertex[i].G; }
	const float& b(int i) const { return vertex[i].B; }
	const float& a(int i) const { return vertex[i].A; }
	float& x(int i) { return vertex[i].pt.x; }
	float& y(int i) { return vertex[i].pt.y; }
	float& z(int i) { return vertex[i].pt.z; }
	float& r(int i) { return vertex[i].R; }
	float& g(int i) { return vertex[i].G; }
	float& b(int i) { return vertex[i].B; }
	float& a(int i) { return vertex[i].A; }

	// These methods are explicitly instantiated for T=float and double.
	void readFrom(mrpt::serialization::CArchive& i);
	void writeTo(mrpt::serialization::CArchive& o) const;
};
#pragma pack(pop)

}  // namespace mrpt::opengl
