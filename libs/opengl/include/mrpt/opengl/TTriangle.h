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
	struct PointNormal
	{
		mrpt::math::TPointXYZfRGBAu8 position;
		mrpt::math::TVector3Df normal;  //!< Must not be normalized
	};

	TTriangle() = default;
	explicit TTriangle(const mrpt::math::TPolygon3D& p)
	{
		ASSERT_EQUAL_(p.size(), 3U);
		for (size_t i = 0; i < 3; i++)
		{
			auto& pp = vertices[i].position;
			pp.pt = p[i];
			pp.r = pp.g = pp.b = 0xff;
		}
		computeNormals();
	}
	explicit TTriangle(
		const mrpt::math::TPoint3Df& p1, const mrpt::math::TPoint3Df& p2,
		const mrpt::math::TPoint3Df& p3)
	{
		vertices[0].position.pt = p1;
		vertices[1].position.pt = p2;
		vertices[2].position.pt = p3;
		computeNormals();
	}

	std::array<PointNormal, 3> vertices;

	const float& x(size_t i) const { return vertices[i].position.pt.x; }
	const float& y(size_t i) const { return vertices[i].position.pt.y; }
	const float& z(size_t i) const { return vertices[i].position.pt.z; }
	const uint8_t& r(size_t i) const { return vertices[i].position.r; }
	const uint8_t& g(size_t i) const { return vertices[i].position.g; }
	const uint8_t& b(size_t i) const { return vertices[i].position.b; }
	const uint8_t& a(size_t i) const { return vertices[i].position.a; }
	float& x(size_t i) { return vertices[i].position.pt.x; }
	float& y(size_t i) { return vertices[i].position.pt.y; }
	float& z(size_t i) { return vertices[i].position.pt.z; }
	uint8_t& r(size_t i) { return vertices[i].position.r; }
	uint8_t& g(size_t i) { return vertices[i].position.g; }
	uint8_t& b(size_t i) { return vertices[i].position.b; }
	uint8_t& a(size_t i) { return vertices[i].position.a; }

	mrpt::math::TPoint3Df& vertex(size_t i) { return vertices[i].position.pt; }
	const mrpt::math::TPoint3Df& vertex(size_t i) const
	{
		return vertices[i].position.pt;
	}

	/** Sets the color of all vertices */
	inline void setColor(const mrpt::img::TColor& c)
	{
		for (size_t i = 0; i < 3; i++)
		{
			vertices[i].position.r = c.R;
			vertices[i].position.g = c.G;
			vertices[i].position.b = c.B;
			vertices[i].position.a = c.A;
		}
	}
	inline void setColor(const mrpt::img::TColorf& c)
	{
		setColor(mrpt::img::TColor(f2u8(c.R), f2u8(c.G), f2u8(c.B), f2u8(c.A)));
	}

	/** Compute the three normals from the cross-product of "v01 x v02".
	 * Note that using this default normals will not lead to interpolated
	 * lighting in the fragment shaders, since all vertex are equal; a derived
	 * class should use custom, more accurate normals to enable soft lighting.
	 */
	void computeNormals();

	// These methods are explicitly instantiated for T=float and double.
	void readFrom(mrpt::serialization::CArchive& i);
	void writeTo(mrpt::serialization::CArchive& o) const;
};
#pragma pack(pop)

}  // namespace mrpt::opengl
