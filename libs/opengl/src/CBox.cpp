/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/math/geometry.h>
#include <mrpt/opengl/CBox.h>
#include <mrpt/opengl/gl_utils.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CBox, CRenderizableShaderTriangles, mrpt::opengl)

CBox::CBox()
	: m_corner_min(-1, -1, -1),
	  m_corner_max(1, 1, 1),

	  m_solidborder_color(0, 0, 0)
{
}

CBox::CBox(
	const mrpt::math::TPoint3D& corner1, const mrpt::math::TPoint3D& corner2,
	bool is_wireframe, float lineWidth)
	: m_wireframe(is_wireframe),
	  m_draw_border(false),
	  m_solidborder_color(0, 0, 0)
{
	CRenderizableShaderWireFrame::setLineWidth(lineWidth);
	setBoxCorners(corner1, corner2);
}

void CBox::render(const RenderContext& rc) const
{
	switch (rc.shader_id)
	{
		case DefaultShaderID::TRIANGLES:
			if (!m_wireframe) CRenderizableShaderTriangles::render(rc);
			break;
		case DefaultShaderID::WIREFRAME:
			if (m_draw_border || m_wireframe)
				CRenderizableShaderWireFrame::render(rc);
			break;
	};
}
void CBox::renderUpdateBuffers() const
{
	CRenderizableShaderTriangles::renderUpdateBuffers();
	CRenderizableShaderWireFrame::renderUpdateBuffers();
}

void CBox::onUpdateBuffers_Wireframe()
{
	auto& vbd = CRenderizableShaderWireFrame::m_vertex_buffer_data;
	auto& cbd = CRenderizableShaderWireFrame::m_color_buffer_data;
	vbd.clear();

	const std::array<mrpt::math::TPoint3D, 2> corner = {m_corner_min,
														m_corner_max};

	for (unsigned int i = 0; i < 2; i++)
	{
		vbd.emplace_back(corner[0].x, corner[0].y, corner[i].z);
		vbd.emplace_back(corner[0].x, corner[1].y, corner[i].z);

		vbd.emplace_back(corner[0].x, corner[1].y, corner[i].z);
		vbd.emplace_back(corner[1].x, corner[1].y, corner[i].z);

		vbd.emplace_back(corner[1].x, corner[1].y, corner[i].z);
		vbd.emplace_back(corner[1].x, corner[0].y, corner[i].z);

		vbd.emplace_back(corner[1].x, corner[0].y, corner[i].z);
		vbd.emplace_back(corner[0].x, corner[0].y, corner[i].z);

		vbd.emplace_back(corner[i].x, corner[0].y, corner[0].z);
		vbd.emplace_back(corner[i].x, corner[0].y, corner[1].z);

		vbd.emplace_back(corner[i].x, corner[1].y, corner[0].z);
		vbd.emplace_back(corner[i].x, corner[1].y, corner[1].z);
	}

	cbd.assign(vbd.size(), m_solidborder_color);
}
void CBox::onUpdateBuffers_Triangles()
{
	auto& tris = CRenderizableShaderTriangles::m_triangles;
	tris.clear();

	const auto &c0 = m_corner_min, &c1 = m_corner_max;
	using P3 = mrpt::math::TPoint3D;

	// Front face:
	tris.emplace_back(
		P3(c1.x, c0.y, c0.z), P3(c0.x, c0.y, c0.z), P3(c1.x, c0.y, c1.z));
	tris.emplace_back(
		P3(c0.x, c0.y, c0.z), P3(c0.x, c0.y, c1.z), P3(c1.x, c0.y, c1.z));

	// Back face:
	tris.emplace_back(
		P3(c1.x, c1.y, c0.z), P3(c0.x, c1.y, c0.z), P3(c1.x, c1.y, c1.z));
	tris.emplace_back(
		P3(c0.x, c1.y, c0.z), P3(c0.x, c1.y, c1.z), P3(c1.x, c1.y, c1.z));

	// Left face:
	tris.emplace_back(
		P3(c0.x, c0.y, c0.z), P3(c0.x, c1.y, c0.z), P3(c0.x, c1.y, c1.z));
	tris.emplace_back(
		P3(c0.x, c0.y, c1.z), P3(c0.x, c0.y, c0.z), P3(c0.x, c1.y, c1.z));

	// Right face:
	tris.emplace_back(
		P3(c1.x, c0.y, c0.z), P3(c1.x, c1.y, c0.z), P3(c1.x, c1.y, c1.z));
	tris.emplace_back(
		P3(c1.x, c0.y, c1.z), P3(c1.x, c0.y, c0.z), P3(c1.x, c1.y, c1.z));

	// Bottom face:
	tris.emplace_back(
		P3(c0.x, c0.y, c0.z), P3(c1.x, c0.y, c0.z), P3(c1.x, c1.y, c0.z));
	tris.emplace_back(
		P3(c0.x, c1.y, c0.z), P3(c0.x, c0.y, c0.z), P3(c1.x, c1.y, c0.z));
	// Top face:

	tris.emplace_back(
		P3(c0.x, c0.y, c1.z), P3(c1.x, c0.y, c1.z), P3(c1.x, c1.y, c1.z));
	tris.emplace_back(
		P3(c0.x, c1.y, c1.z), P3(c0.x, c0.y, c1.z), P3(c1.x, c1.y, c1.z));

	// All faces, all vertices, same color:
	for (auto& t : tris)
		for (unsigned int i = 0; i < 3; i++)
		{
			t.vertex[i].R = u8tof(m_color.R);
			t.vertex[i].G = u8tof(m_color.G);
			t.vertex[i].B = u8tof(m_color.B);
			t.vertex[i].A = u8tof(m_color.A);
		}
}

uint8_t CBox::serializeGetVersion() const { return 1; }
void CBox::serializeTo(mrpt::serialization::CArchive& out) const
{
	writeToStreamRender(out);
	// version 0
	out << m_corner_min.x << m_corner_min.y << m_corner_min.z << m_corner_max.x
		<< m_corner_max.y << m_corner_max.z << m_wireframe << m_lineWidth;
	// Version 1:
	out << m_draw_border << m_solidborder_color;
}

void CBox::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		case 1:
			readFromStreamRender(in);
			in >> m_corner_min.x >> m_corner_min.y >> m_corner_min.z >>
				m_corner_max.x >> m_corner_max.y >> m_corner_max.z >>
				m_wireframe >> m_lineWidth;
			// Version 1:
			if (version >= 1)
				in >> m_draw_border >> m_solidborder_color;
			else
			{
				m_draw_border = false;
			}

			break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
	CRenderizable::notifyChange();
}

void CBox::setBoxCorners(
	const mrpt::math::TPoint3D& corner1, const mrpt::math::TPoint3D& corner2)
{
	CRenderizable::notifyChange();

	// Order the coordinates so we always have the min/max in their right
	// position:
	m_corner_min.x = std::min(corner1.x, corner2.x);
	m_corner_min.y = std::min(corner1.y, corner2.y);
	m_corner_min.z = std::min(corner1.z, corner2.z);

	m_corner_max.x = std::max(corner1.x, corner2.x);
	m_corner_max.y = std::max(corner1.y, corner2.y);
	m_corner_max.z = std::max(corner1.z, corner2.z);
}

bool CBox::traceRay(const mrpt::poses::CPose3D& o, double& dist) const
{
	MRPT_UNUSED_PARAM(o);
	MRPT_UNUSED_PARAM(dist);
	THROW_EXCEPTION("TO DO");
}

void CBox::getBoundingBox(
	mrpt::math::TPoint3D& bb_min, mrpt::math::TPoint3D& bb_max) const
{
	bb_min = m_corner_min;
	bb_max = m_corner_max;

	// Convert to coordinates of my parent:
	m_pose.composePoint(bb_min, bb_min);
	m_pose.composePoint(bb_max, bb_max);
}
