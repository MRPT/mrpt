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

	// Line loop:
	const auto N = pts.size();
	for (size_t i = 0; i < N; i++)
	{
		const auto ip = (i + 1) % N;
		vbd.emplace_back(pts[i][0], pts[i][1], .0f);
		vbd.emplace_back(pts[ip][0], pts[ip][1], .0f);
	}

	// All lines, same color:
	cbd.assign(vbd.size(), m_color);
}

template <>
void CGeneralizedEllipsoidTemplate<3>::implUpdate_Wireframe()
{
	const auto& pts = m_render_pts;

	auto& vbd = CRenderizableShaderWireFrame::m_vertex_buffer_data;
	auto& cbd = CRenderizableShaderWireFrame::m_color_buffer_data;
	vbd.clear();

	const auto slices = m_numSegments, stacks = m_numSegments;

	// Points in the ellipsoid:
	//  * "#slices" slices, with "#stacks" points each, but for the two ends
	//  * 1 point at each end slice
	// #total points = stacks*(slices-2) + 2
	ASSERT_EQUAL_((slices - 2) * stacks + 2, pts.size());
	const size_t idx_1st_slice = 1;

	// 1st slice: triangle fan (if it were solid)
	// ----------------------------
	for (size_t i = 0; i < stacks; i++)
	{
		const auto idx = idx_1st_slice + i;
		vbd.emplace_back(pts[0]);
		vbd.emplace_back(pts[idx]);
	}

	// Middle slices: triangle strip (if it were solid)
	// ----------------------------
	for (size_t s = 0; s < slices - 3; s++)
	{
		size_t idx0 = idx_1st_slice + stacks * s;
		size_t idx1 = idx0 + stacks;

		for (size_t i = 0; i < stacks; i++)
		{
			const size_t ii =
				(i == (stacks - 1) ? 0 : i + 1);  // next i with wrapping

			vbd.emplace_back(pts[idx0 + i]);
			vbd.emplace_back(pts[idx1 + ii]);

			vbd.emplace_back(pts[idx1 + ii]);
			vbd.emplace_back(pts[idx1 + i]);

			vbd.emplace_back(pts[idx1 + i]);
			vbd.emplace_back(pts[idx0 + i]);

			vbd.emplace_back(pts[idx0 + i]);
			vbd.emplace_back(pts[idx0 + ii]);

			vbd.emplace_back(pts[idx0 + ii]);
			vbd.emplace_back(pts[idx1 + ii]);
		}
	}

	// Last slice: triangle fan (if it were solid)
	// ----------------------------
	const size_t idxN = pts.size() - 1;
	const size_t idx_last_slice = idx_1st_slice + (slices - 3) * stacks;

	for (size_t i = 0; i < stacks; i++)
	{
		vbd.emplace_back(pts[idxN]);
		vbd.emplace_back(pts[idx_last_slice + i]);
	}

	// All lines, same color:
	cbd.assign(vbd.size(), m_color);
}

template <>
void CGeneralizedEllipsoidTemplate<2>::implUpdate_Triangles()
{
	using P3f = mrpt::math::TPoint3Df;
	const auto& pts = m_render_pts;

	// Render precomputed points in m_render_pts:
	auto& tris = CRenderizableShaderTriangles::m_triangles;
	tris.clear();

	const auto N = pts.size();
	for (size_t i = 0; i < N; i++)
	{
		const auto ip = (i + 1) % N;
		tris.emplace_back(
			P3f(0, 0, 0), P3f(pts[i][0], pts[i][1], .0f),
			P3f(pts[ip][0], pts[ip][1], .0f));
	}

	// All faces, all vertices, same color:
	for (auto& t : tris) t.setColor(m_color);
}

template <>
void CGeneralizedEllipsoidTemplate<3>::implUpdate_Triangles()
{
	using P3f = mrpt::math::TPoint3Df;
	const auto& pts = m_render_pts;

	// Render precomputed points in m_render_pts:
	auto& tris = CRenderizableShaderTriangles::m_triangles;
	tris.clear();

	const auto slices = m_numSegments, stacks = m_numSegments;

	// Points in the ellipsoid:
	//  * "#slices" slices, with "#stacks" points each, but for the two ends
	//  * 1 point at each end slice
	// #total points = stacks*(slices-2) + 2
	ASSERT_EQUAL_((slices - 2) * stacks + 2, pts.size());
	const size_t idx_1st_slice = 1;

	// 1st slice: triangle fan (if it were solid)
	// ----------------------------
	for (size_t i = 0; i < stacks; i++)
	{
		const auto idx = idx_1st_slice + i;
		const auto idxp = idx_1st_slice + ((i + 1) % stacks);

		tris.emplace_back(
			// Points
			pts[0], pts[idx], pts[idxp],
			// Normals:
			pts[0], pts[idx], pts[idxp]);
	}

	// Middle slices: triangle strip (if it were solid)
	// ----------------------------
	for (size_t s = 0; s < slices - 3; s++)
	{
		size_t idx0 = idx_1st_slice + stacks * s;
		size_t idx1 = idx0 + stacks;

		for (size_t i = 0; i < stacks; i++)
		{
			const size_t ii =
				(i == (stacks - 1) ? 0 : i + 1);  // next i with wrapping

			tris.emplace_back(
				// Points
				pts[idx0 + i], pts[idx0 + ii], pts[idx1 + i],
				// Normals:
				pts[idx0 + i], pts[idx0 + ii], pts[idx1 + i]);
			tris.emplace_back(
				// Points
				pts[idx1 + ii], pts[idx1 + i], pts[idx0 + ii],
				// Normals:
				pts[idx1 + ii], pts[idx1 + i], pts[idx0 + ii]);
		}
	}

	// Last slice: triangle fan (if it were solid)
	// ----------------------------
	const size_t idxN = pts.size() - 1;
	const size_t idx_last_slice = idx_1st_slice + (slices - 3) * stacks;

	for (size_t i = 0; i < stacks; i++)
	{
		const auto idx = idx_last_slice + i;
		const auto idxp = idx_last_slice + ((i + 1) % stacks);

		tris.emplace_back(
			// Points
			pts[idx], pts[idxN], pts[idxp],
			// Normals
			pts[idx], pts[idxN], pts[idxp]);
	}

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
