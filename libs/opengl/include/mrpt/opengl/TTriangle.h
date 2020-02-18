/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/core/bits_math.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/img/TColor.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPolygon3D.h>
#include <mrpt/serialization/serialization_frwds.h>

namespace mrpt::opengl
{
// Ensure 1-byte memory alignment, no additional stride bytes.
#pragma pack(push, 1)

/** A triangle (float coordinates) with RGBA colors (u8) for each vertex.
 * The structure is memory packed to 1-byte, to ensure it can be used in GPU
 * memory vertex arrays without unexpected paddings.
 *
 * \sa opengl::COpenGLScene, CSetOfTexturedTriangles
 * \ingroup mrpt_opengl_grp
 */
struct TTriangle
{
	TTriangle() = default;
	explicit TTriangle(const mrpt::math::TPolygon3D& p)
	{
		ASSERT_EQUAL_(p.size(), 3U);
		for (size_t i = 0; i < 3; i++)
		{
			vertex[i].pt = p[i];
			vertex[i].r = vertex[i].g = vertex[i].b = 0xff;
		}
	}
	explicit TTriangle(
		const mrpt::math::TPoint3Df& p1, const mrpt::math::TPoint3Df& p2,
		const mrpt::math::TPoint3Df& p3)
	{
		vertex[0].pt = p1;
		vertex[1].pt = p2;
		vertex[2].pt = p3;
	}

	mrpt::math::TPointXYZfRGBAu8 vertex[3];

	const float& x(int i) const { return vertex[i].pt.x; }
	const float& y(int i) const { return vertex[i].pt.y; }
	const float& z(int i) const { return vertex[i].pt.z; }
	const uint8_t& r(int i) const { return vertex[i].r; }
	const uint8_t& g(int i) const { return vertex[i].g; }
	const uint8_t& b(int i) const { return vertex[i].b; }
	const uint8_t& a(int i) const { return vertex[i].a; }
	float& x(int i) { return vertex[i].pt.x; }
	float& y(int i) { return vertex[i].pt.y; }
	float& z(int i) { return vertex[i].pt.z; }
	uint8_t& r(int i) { return vertex[i].r; }
	uint8_t& g(int i) { return vertex[i].g; }
	uint8_t& b(int i) { return vertex[i].b; }
	uint8_t& a(int i) { return vertex[i].a; }

	/** Sets the color of all vertices */
	inline void setColor(const mrpt::img::TColor& c)
	{
		for (size_t i = 0; i < 3; i++)
		{
			vertex[i].r = c.R;
			vertex[i].g = c.G;
			vertex[i].b = c.B;
			vertex[i].a = c.A;
		}
	}
	inline void setColor(const mrpt::img::TColorf& c)
	{
		setColor(mrpt::img::TColor(f2u8(c.R), f2u8(c.G), f2u8(c.B), f2u8(c.A)));
	}

	// These methods are explicitly instantiated for T=float and double.
	void readFrom(mrpt::serialization::CArchive& i);
	void writeTo(mrpt::serialization::CArchive& o) const;
};
#pragma pack(pop)

}  // namespace mrpt::opengl
