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
#include <mrpt/math/TPolygon3D.h>

namespace mrpt::opengl
{
// Ensure 1-byte memory alignment, no additional stride bytes.
#pragma pack(push, 1)

/** A triangle with RGBA colors for each vertex.
 * The structure is memory packed to 1-byte, to ensure it can be used in GPU
 * memory vertex arrays without unexpected paddings.
 *
 * \sa opengl::COpenGLScene, CSetOfTexturedTriangles
 * \ingroup mrpt_opengl_grp
 */
template <typename T>
struct TTriangle_
{
	TTriangle_() = default;

	TTriangle_(const mrpt::math::TPolygon3D& p)
	{
		ASSERT_EQUAL_(p.size(), 3U);
		for (size_t i = 0; i < 3; i++)
		{
			x[i] = p[i].x;
			y[i] = p[i].y;
			z[i] = p[i].z;
			r[i] = g[i] = b[i] = a[i] = 1;
		}
	}
	T x[3] = {0, 0, 0}, y[3] = {0, 0, 0}, z[3] = {0, 0, 0};
	T r[3] = {1, 1, 1}, g[3] = {1, 1, 1}, b[3] = {1, 1, 1}, a[3] = {1, 1, 1};
};
#pragma pack(pop)

/** Triangle with float XYZ+RGBA vertices. \sa TTriangle_ */
using TTriangle = TTriangle_<float>;

}  // namespace mrpt::opengl
