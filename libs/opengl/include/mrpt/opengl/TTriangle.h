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
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPolygon3D.h>
#include <mrpt/serialization/serialization_frwds.h>
#include <array>

namespace mrpt::opengl
{
// Ensure 1-byte memory alignment, no additional stride bytes.
#pragma pack(push, 1)

/** A triangle (float coordinates) with RGBA colors (u8) and UV (texture
 * coordinates) for each vertex. Note that not all the fields must be filled in,
 * it depends on the consumer of the structure.
 *
 * The structure is memory packed to 1-byte, to ensure it can be used in GPU
 * memory vertex arrays without unexpected paddings.
 *
 * \sa opengl::COpenGLScene, CSetOfTexturedTriangles
 * \ingroup mrpt_opengl_grp
 */
struct TTriangle
{
	struct Vertex
	{
		mrpt::math::TPointXYZfRGBAu8 xyzrgba;
		mrpt::math::TVector3Df normal;  //!< Must not be normalized
		mrpt::math::TVector2Df uv;  //!< texture coordinates (0,0)-(1,1)

		void setColor(const mrpt::img::TColor& c)
		{
			xyzrgba.r = c.R;
			xyzrgba.g = c.G;
			xyzrgba.b = c.B;
			xyzrgba.a = c.A;
		}
	};

	TTriangle() = default;
	explicit TTriangle(const mrpt::math::TPolygon3D& p)
	{
		ASSERT_EQUAL_(p.size(), 3U);
		for (size_t i = 0; i < 3; i++)
		{
			auto& pp = vertices[i].xyzrgba;
			pp.pt = p[i];
			pp.r = pp.g = pp.b = 0xff;
		}
		computeNormals();
	}
	/** Constructor from 3 points (default normals are computed) */
	explicit TTriangle(
		const mrpt::math::TPoint3Df& p1, const mrpt::math::TPoint3Df& p2,
		const mrpt::math::TPoint3Df& p3)
	{
		vertices[0].xyzrgba.pt = p1;
		vertices[1].xyzrgba.pt = p2;
		vertices[2].xyzrgba.pt = p3;
		computeNormals();
	}
	/** Constructor from 3 points and its 3 normals */
	explicit TTriangle(
		const mrpt::math::TPoint3Df& p1, const mrpt::math::TPoint3Df& p2,
		const mrpt::math::TPoint3Df& p3, const mrpt::math::TVector3Df& n1,
		const mrpt::math::TVector3Df& n2, const mrpt::math::TVector3Df& n3)
	{
		vertices[0].xyzrgba.pt = p1;
		vertices[0].normal = n1;
		vertices[1].xyzrgba.pt = p2;
		vertices[1].normal = n2;
		vertices[2].xyzrgba.pt = p3;
		vertices[2].normal = n3;
	}

	std::array<Vertex, 3> vertices;

	const float& x(size_t i) const { return vertices[i].xyzrgba.pt.x; }
	const float& y(size_t i) const { return vertices[i].xyzrgba.pt.y; }
	const float& z(size_t i) const { return vertices[i].xyzrgba.pt.z; }
	const uint8_t& r(size_t i) const { return vertices[i].xyzrgba.r; }
	const uint8_t& g(size_t i) const { return vertices[i].xyzrgba.g; }
	const uint8_t& b(size_t i) const { return vertices[i].xyzrgba.b; }
	const uint8_t& a(size_t i) const { return vertices[i].xyzrgba.a; }
	const float& u(size_t i) const { return vertices[i].uv.x; }
	const float& v(size_t i) const { return vertices[i].uv.y; }
	float& x(size_t i) { return vertices[i].xyzrgba.pt.x; }
	float& y(size_t i) { return vertices[i].xyzrgba.pt.y; }
	float& z(size_t i) { return vertices[i].xyzrgba.pt.z; }
	uint8_t& r(size_t i) { return vertices[i].xyzrgba.r; }
	uint8_t& g(size_t i) { return vertices[i].xyzrgba.g; }
	uint8_t& b(size_t i) { return vertices[i].xyzrgba.b; }
	uint8_t& a(size_t i) { return vertices[i].xyzrgba.a; }
	float& u(size_t i) { return vertices[i].uv.x; }
	float& v(size_t i) { return vertices[i].uv.y; }

	mrpt::math::TPoint3Df& vertex(size_t i) { return vertices[i].xyzrgba.pt; }
	const mrpt::math::TPoint3Df& vertex(size_t i) const
	{
		return vertices[i].xyzrgba.pt;
	}

	/** Sets the color of all vertices */
	inline void setColor(const mrpt::img::TColor& c)
	{
		for (size_t i = 0; i < 3; i++)
		{
			vertices[i].xyzrgba.r = c.R;
			vertices[i].xyzrgba.g = c.G;
			vertices[i].xyzrgba.b = c.B;
			vertices[i].xyzrgba.a = c.A;
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
