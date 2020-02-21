/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/opengl/CGeneralizedEllipsoidTemplate.h>
#include <mrpt/opengl/gl_utils.h>
#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace std;

template <>
void CGeneralizedEllipsoidTemplate<2>::implUpdate_Wireframe()
{
	const auto& pts = m_render_pts;

	auto& vbd = CRenderizableShaderWireFrame::m_vertex_buffer_data;
	auto& cbd = CRenderizableShaderWireFrame::m_color_buffer_data;
	vbd.clear();
	// All lines, same color:
	cbd.assign(vbd.size(), m_color);

	// Line loop:
	const auto N = pts.size();
	for (size_t i = 0; i < N; i++)
	{
		const auto ip = (i + 1) % N;
		vbd.emplace_back(pts[i][0], pts[i][1], .0f);
		vbd.emplace_back(pts[ip][0], pts[ip][1], .0f);
	}
}

template <>
void CGeneralizedEllipsoidTemplate<3>::implUpdate_Wireframe()
{
#if 0
	glEnable(GL_BLEND);
	CHECK_OPENGL_ERROR();
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	CHECK_OPENGL_ERROR();
	glLineWidth(lineWidth);
	CHECK_OPENGL_ERROR();
	glDisable(GL_LIGHTING);  // Disable lights when drawing lines

	// Points in the ellipsoid:
	//  * "#slices" slices, with "#stacks" points each, but for the two ends
	//  * 1 point at each end slice
	// #total points = stacks*(slices-2) + 2
	ASSERT_EQUAL_((slices - 2) * stacks + 2, pts.size());
	const size_t idx_1st_slice = 1;

	// 1st slice: triangle fan (if it were solid)
	// ----------------------------
	glBegin(GL_LINES);
	for (size_t i = 0; i < stacks; i++)
	{
		glVertex3fv(&pts[0][0]);
		glVertex3fv(&pts[idx_1st_slice + i][0]);
	}
	glEnd();

	// Middle slices: triangle strip (if it were solid)
	// ----------------------------
	for (size_t s = 0; s < slices - 3; s++)
	{
		size_t idx_this_slice = idx_1st_slice + stacks * s;
		size_t idx_next_slice = idx_this_slice + stacks;

		for (size_t i = 0; i < stacks; i++)
		{
			const size_t ii =
				(i == (stacks - 1) ? 0 : i + 1);  // next i with wrapping

			glBegin(GL_LINE_STRIP);
			glVertex3fv(&pts[idx_this_slice + i][0]);
			glVertex3fv(&pts[idx_next_slice + ii][0]);
			glVertex3fv(&pts[idx_next_slice + i][0]);
			glVertex3fv(&pts[idx_this_slice + i][0]);
			glVertex3fv(&pts[idx_this_slice + ii][0]);
			glVertex3fv(&pts[idx_next_slice + ii][0]);
			glEnd();
		}
	}

	// Last slice: triangle fan (if it were solid)
	// ----------------------------
	const size_t idx_last_pt = pts.size() - 1;
	const size_t idx_last_slice = idx_1st_slice + (slices - 3) * stacks;
	glBegin(GL_LINES);
	for (size_t i = 0; i < stacks; i++)
	{
		glVertex3fv(&pts[idx_last_pt][0]);
		glVertex3fv(&pts[idx_last_slice + i][0]);
	}
	glEnd();

	// glBegin( GL_POINTS );
	// const size_t N = pts.size();
	// for (size_t i=0;i<N;i++)
	//	glVertex3f( pts[i][0], pts[i][1], pts[i][2] );
	// glEnd();

	glDisable(GL_BLEND);
#endif
}

template <>
void CGeneralizedEllipsoidTemplate<2>::implUpdate_Triangles()
{
	using P3f = mrpt::math::TPoint3Df;

	// Render precomputed points in m_render_pts:
	auto& tris = CRenderizableShaderTriangles::m_triangles;
	tris.clear();

	// All faces, all vertices, same color:
	for (auto& t : tris) t.setColor(m_color);
}

template <>
void CGeneralizedEllipsoidTemplate<3>::implUpdate_Triangles()
{
	// Render precomputed points in m_render_pts:
	auto& tris = CRenderizableShaderTriangles::m_triangles;
	tris.clear();

	// All faces, all vertices, same color:
	for (auto& t : tris) t.setColor(m_color);
}

template <>
void CGeneralizedEllipsoidTemplate<2>::generatePoints(
	const CGeneralizedEllipsoidTemplate<2>::cov_matrix_t& U,
	std::vector<CGeneralizedEllipsoidTemplate<2>::array_parameter_t>& pts) const
{
	pts.clear();
	pts.reserve(m_numSegments);
	const double Aa = 2 * M_PI / m_numSegments;
	for (double ang = 0; ang < 2 * M_PI; ang += Aa)
	{
		const double ccos = cos(ang);
		const double ssin = sin(ang);

		pts.resize(pts.size() + 1);

		auto& pt = pts.back();

		pt[0] = d2f(m_mean[0] + ccos * U(0, 0) + ssin * U(0, 1));
		pt[1] = d2f(m_mean[1] + ccos * U(1, 0) + ssin * U(1, 1));
	}
}

inline void aux_add3DpointWithEigenVectors(
	const double x, const double y, const double z,
	std::vector<mrpt::math::CMatrixFixed<float, 3, 1>>& pts,
	const mrpt::math::CMatrixFixed<double, 3, 3>& M,
	const mrpt::math::CMatrixFixed<double, 3, 1>& mean)
{
	pts.resize(pts.size() + 1);
	mrpt::math::CMatrixFixed<float, 3, 1>& pt = pts.back();
	pt[0] = d2f(mean[0] + x * M(0, 0) + y * M(0, 1) + z * M(0, 2));
	pt[1] = d2f(mean[1] + x * M(1, 0) + y * M(1, 1) + z * M(1, 2));
	pt[2] = d2f(mean[2] + x * M(2, 0) + y * M(2, 1) + z * M(2, 2));
}

template <>
void CGeneralizedEllipsoidTemplate<3>::generatePoints(
	const CGeneralizedEllipsoidTemplate<3>::cov_matrix_t& U,
	std::vector<CGeneralizedEllipsoidTemplate<3>::array_parameter_t>& pts) const
{
	MRPT_START
	const auto slices = m_numSegments, stacks = m_numSegments;
	ASSERT_ABOVEEQ_(slices, 3);
	ASSERT_ABOVEEQ_(stacks, 3);
	// sin/cos cache --------
	// Slices: [0,pi]
	std::vector<double> slice_cos(slices), slice_sin(slices);
	for (uint32_t i = 0; i < slices; i++)
	{
		double angle = M_PI * i / double(slices - 1);
		slice_sin[i] = sin(angle);
		slice_cos[i] = cos(angle);
	}
	// Stacks: [0,2*pi]
	std::vector<double> stack_sin(stacks), stack_cos(stacks);
	for (uint32_t i = 0; i < stacks; i++)
	{
		double angle = 2 * M_PI * i / double(stacks);
		stack_sin[i] = sin(angle);
		stack_cos[i] = cos(angle);
	}

	// Points in the ellipsoid:
	//  * "#slices" slices, with "#stacks" points each, but for the two ends
	//  * 1 point at each end slice
	// #total points = stacks*(slices-2) + 2
	pts.clear();
	pts.reserve((slices - 2) * stacks + 2);

	for (uint32_t i = 0; i < slices; i++)
	{
		if (i == 0)
			aux_add3DpointWithEigenVectors(1, 0, 0, pts, U, m_mean);
		else if (i == (slices - 1))
			aux_add3DpointWithEigenVectors(-1, 0, 0, pts, U, m_mean);
		else
		{
			const double x = slice_cos[i];
			const double R = slice_sin[i];

			for (uint32_t j = 0; j < stacks; j++)
			{
				const double y = R * stack_cos[j];
				const double z = R * stack_sin[j];
				aux_add3DpointWithEigenVectors(x, y, z, pts, U, m_mean);
			}
		}
	}

	MRPT_END
}
