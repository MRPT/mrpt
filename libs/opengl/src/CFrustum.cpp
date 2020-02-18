/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/opengl/CFrustum.h>
#include <mrpt/opengl/gl_utils.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CFrustum, CRenderizableShaderTriangles, mrpt::opengl)

void CFrustum::render(const RenderContext& rc) const
{
	switch (rc.shader_id)
	{
		case DefaultShaderID::TRIANGLES:
			if (m_draw_planes) CRenderizableShaderTriangles::render(rc);
			break;
		case DefaultShaderID::WIREFRAME:
			if (m_draw_lines) CRenderizableShaderWireFrame::render(rc);
			break;
	};
}
void CFrustum::renderUpdateBuffers() const
{
	CRenderizableShaderTriangles::renderUpdateBuffers();
	CRenderizableShaderWireFrame::renderUpdateBuffers();
}

std::array<mrpt::math::TPoint3Df, 8> CFrustum::computeFrustumCorners() const
{
	std::array<mrpt::math::TPoint3Df, 8> pts;
	for (size_t j = 0; j < 2; j++)
	{
		const float r = j == 0 ? m_min_distance : m_max_distance;
		for (size_t i = 0; i < 4; i++) pts[4 * j + i].x = r;
		pts[4 * j + 0].y = -r * tan(m_fov_horz_left);
		pts[4 * j + 1].y = -r * tan(m_fov_horz_left);
		pts[4 * j + 2].y = r * tan(m_fov_horz_right);
		pts[4 * j + 3].y = r * tan(m_fov_horz_right);
		pts[4 * j + 0].z = -r * tan(m_fov_vert_down);
		pts[4 * j + 1].z = r * tan(m_fov_vert_up);
		pts[4 * j + 2].z = -r * tan(m_fov_vert_down);
		pts[4 * j + 3].z = r * tan(m_fov_vert_up);
	}
	return pts;
}

void CFrustum::onUpdateBuffers_Wireframe()
{
	auto& vbd = CRenderizableShaderWireFrame::m_vertex_buffer_data;
	auto& cbd = CRenderizableShaderWireFrame::m_color_buffer_data;
	vbd.clear();

	const std::array<mrpt::math::TPoint3Df, 8> pts = computeFrustumCorners();

	const std::array<int, 16> draw_path = {0, 1, 3, 2, 0, 4, 6, 2,
										   3, 7, 6, 4, 5, 7, 5, 1};

	// GL_LINE_STRIP:
	for (unsigned int idx = 0; idx < draw_path.size(); idx++)
	{
		const unsigned int idx_next = (idx + 1) % draw_path.size();
		vbd.emplace_back(pts[draw_path[idx]]);
		vbd.emplace_back(pts[draw_path[idx_next]]);
	}

	cbd.assign(vbd.size(), m_color);
}
void CFrustum::onUpdateBuffers_Triangles()
{
	auto& tris = CRenderizableShaderTriangles::m_triangles;
	tris.clear();

	const std::array<mrpt::math::TPoint3Df, 8> pts = computeFrustumCorners();

	tris.emplace_back(pts[0], pts[2], pts[6]);
	tris.emplace_back(pts[6], pts[4], pts[0]);

	tris.emplace_back(pts[2], pts[3], pts[7]);
	tris.emplace_back(pts[7], pts[6], pts[2]);

	tris.emplace_back(pts[4], pts[6], pts[7]);
	tris.emplace_back(pts[7], pts[5], pts[4]);

	tris.emplace_back(pts[1], pts[5], pts[7]);
	tris.emplace_back(pts[7], pts[3], pts[1]);

	tris.emplace_back(pts[1], pts[5], pts[7]);
	tris.emplace_back(pts[7], pts[3], pts[1]);

	tris.emplace_back(pts[4], pts[5], pts[1]);
	tris.emplace_back(pts[1], pts[0], pts[4]);

	// All faces, all vertices, same color:
	for (auto& t : tris) t.setColor(m_planes_color);
}

// Ctors
CFrustum::CFrustum()
	: m_fov_horz_left(mrpt::DEG2RAD(45.0f)),
	  m_fov_horz_right(mrpt::DEG2RAD(45.0f)),
	  m_fov_vert_down(mrpt::DEG2RAD(30.0f)),
	  m_fov_vert_up(mrpt::DEG2RAD(30.0f)),
	  m_planes_color(0xE0, 0x00, 0x00, 0x50)  // RGBA
{
	keep_min(m_fov_horz_left, DEG2RAD(89.9f));
	keep_max(m_fov_horz_left, 0);
	keep_min(m_fov_horz_right, DEG2RAD(89.9f));
	keep_max(m_fov_horz_right, 0);
	keep_min(m_fov_vert_down, DEG2RAD(89.9f));
	keep_max(m_fov_vert_down, 0);
	keep_min(m_fov_vert_up, DEG2RAD(89.9f));
	keep_max(m_fov_vert_up, 0);
}

CFrustum::CFrustum(
	float near_distance, float far_distance, float horz_FOV_degrees,
	float vert_FOV_degrees, float lineWidth, bool draw_lines, bool draw_planes)
	: m_min_distance(near_distance),
	  m_max_distance(far_distance),
	  m_fov_horz_left(mrpt::DEG2RAD(.5f * horz_FOV_degrees)),
	  m_fov_horz_right(mrpt::DEG2RAD(.5f * horz_FOV_degrees)),
	  m_fov_vert_down(mrpt::DEG2RAD(.5f * vert_FOV_degrees)),
	  m_fov_vert_up(mrpt::DEG2RAD(.5f * vert_FOV_degrees)),
	  m_draw_lines(draw_lines),
	  m_draw_planes(draw_planes),
	  m_planes_color(0xE0, 0x00, 0x00, 0x50)  // RGBA
{
	this->setLineWidth(lineWidth);
}

uint8_t CFrustum::serializeGetVersion() const { return 0; }
void CFrustum::serializeTo(mrpt::serialization::CArchive& out) const
{
	writeToStreamRender(out);
	// version 0
	out << m_min_distance << m_max_distance << m_fov_horz_left
		<< m_fov_horz_right << m_fov_vert_down << m_fov_vert_up << m_draw_lines
		<< m_draw_planes << m_lineWidth << m_planes_color.R << m_planes_color.G
		<< m_planes_color.B << m_planes_color.A;
}

void CFrustum::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
			readFromStreamRender(in);
			in >> m_min_distance >> m_max_distance >> m_fov_horz_left >>
				m_fov_horz_right >> m_fov_vert_down >> m_fov_vert_up >>
				m_draw_lines >> m_draw_planes >> m_lineWidth >>
				m_planes_color.R >> m_planes_color.G >> m_planes_color.B >>
				m_planes_color.A;
			break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
	CRenderizable::notifyChange();
}

bool CFrustum::traceRay(const mrpt::poses::CPose3D& o, double& dist) const
{
	MRPT_UNUSED_PARAM(o);
	MRPT_UNUSED_PARAM(dist);
	THROW_EXCEPTION("TO DO");
}

// setters:
void CFrustum::setNearFarPlanes(
	const float near_distance, const float far_distance)
{
	m_min_distance = near_distance;
	m_max_distance = far_distance;
	CRenderizable::notifyChange();
}
void CFrustum::setHorzFOV(const float fov_horz_degrees)
{
	m_fov_horz_right = m_fov_horz_left = 0.5f * mrpt::DEG2RAD(fov_horz_degrees);
	keep_min(m_fov_horz_left, DEG2RAD(89.9f));
	keep_max(m_fov_horz_left, 0);
	keep_min(m_fov_horz_right, DEG2RAD(89.9f));
	keep_max(m_fov_horz_right, 0);
	CRenderizable::notifyChange();
}
void CFrustum::setVertFOV(const float fov_vert_degrees)
{
	m_fov_vert_down = m_fov_vert_up = 0.5f * mrpt::DEG2RAD(fov_vert_degrees);
	keep_min(m_fov_vert_down, DEG2RAD(89.9f));
	keep_max(m_fov_vert_down, 0);
	keep_min(m_fov_vert_up, DEG2RAD(89.9f));
	keep_max(m_fov_vert_up, 0);
	CRenderizable::notifyChange();
}
void CFrustum::setHorzFOVAsymmetric(
	const float fov_horz_left_degrees, const float fov_horz_right_degrees)
{
	m_fov_horz_left = mrpt::DEG2RAD(fov_horz_left_degrees);
	m_fov_horz_right = mrpt::DEG2RAD(fov_horz_right_degrees);
	keep_min(m_fov_horz_left, DEG2RAD(89.9f));
	keep_max(m_fov_horz_left, 0);
	keep_min(m_fov_horz_right, DEG2RAD(89.9f));
	keep_max(m_fov_horz_right, 0);
	CRenderizable::notifyChange();
}
void CFrustum::setVertFOVAsymmetric(
	const float fov_vert_down_degrees, const float fov_vert_up_degrees)
{
	m_fov_vert_down = mrpt::DEG2RAD(fov_vert_down_degrees);
	m_fov_vert_up = mrpt::DEG2RAD(fov_vert_up_degrees);
	keep_min(m_fov_vert_down, DEG2RAD(89.9f));
	keep_max(m_fov_vert_down, 0);
	keep_min(m_fov_vert_up, DEG2RAD(89.9f));
	keep_max(m_fov_vert_up, 0);
	CRenderizable::notifyChange();
}

void CFrustum::getBoundingBox(
	mrpt::math::TPoint3D& bb_min, mrpt::math::TPoint3D& bb_max) const
{
	// Compute the 8 corners of the frustum:
	TPoint3Df pts[8];
	for (int j = 0; j < 2; j++)
	{
		const float r = j == 0 ? m_min_distance : m_max_distance;
		for (int i = 0; i < 4; i++) pts[4 * j + i].x = r;
		pts[4 * j + 0].y = -r * tan(m_fov_horz_left);
		pts[4 * j + 1].y = -r * tan(m_fov_horz_left);
		pts[4 * j + 2].y = r * tan(m_fov_horz_right);
		pts[4 * j + 3].y = r * tan(m_fov_horz_right);
		pts[4 * j + 0].z = -r * tan(m_fov_vert_down);
		pts[4 * j + 1].z = r * tan(m_fov_vert_up);
		pts[4 * j + 2].z = -r * tan(m_fov_vert_down);
		pts[4 * j + 3].z = r * tan(m_fov_vert_up);
	}

	bb_min = TPoint3D(
		std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
		std::numeric_limits<double>::max());
	bb_max = TPoint3D(
		-std::numeric_limits<double>::max(),
		-std::numeric_limits<double>::max(),
		-std::numeric_limits<double>::max());
	for (auto& pt : pts)
	{
		keep_min(bb_min.x, pt.x);
		keep_min(bb_min.y, pt.y);
		keep_min(bb_min.z, pt.z);

		keep_max(bb_max.x, pt.x);
		keep_max(bb_max.y, pt.y);
		keep_max(bb_max.z, pt.z);
	}

	// Convert to coordinates of my parent:
	m_pose.composePoint(bb_min, bb_min);
	m_pose.composePoint(bb_max, bb_max);
}
