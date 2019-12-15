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

#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::opengl;

using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CFrustum, CRenderizableDisplayList, mrpt::opengl)

/*---------------------------------------------------------------
							render
  ---------------------------------------------------------------*/
void CFrustum::render_dl() const
{
#if MRPT_HAS_OPENGL_GLUT
	if (m_color.A != 255 || (m_draw_planes && m_planes_color.A != 255))
	{
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	}
	else
	{
		glEnable(GL_DEPTH_TEST);
		glDisable(GL_BLEND);
	}

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

	// Render lines:
	if (m_draw_lines)
	{
		glDisable(GL_LIGHTING);  // Disable lights when drawing lines

		const int draw_path[] = {0, 1, 3, 2, 0, 4, 6, 2,
								 3, 7, 6, 4, 5, 7, 5, 1};

		// wireframe:
		glLineWidth(m_lineWidth);
		CHECK_OPENGL_ERROR();
		glBegin(GL_LINE_STRIP);
		glColor4ub(m_color.R, m_color.G, m_color.B, m_color.A);

		for (int i : draw_path) glVertex3fv(&pts[i].x);

		glEnd();

		glEnable(GL_LIGHTING);  // Disable lights when drawing lines
	}

	if (m_draw_planes)
	{
		// solid:
		glBegin(GL_TRIANGLES);
		glColor4ub(
			m_planes_color.R, m_planes_color.G, m_planes_color.B,
			m_planes_color.A);

		gl_utils::renderQuadWithNormal(pts[0], pts[2], pts[6], pts[4]);
		gl_utils::renderQuadWithNormal(pts[2], pts[3], pts[7], pts[6]);
		gl_utils::renderQuadWithNormal(pts[4], pts[6], pts[7], pts[5]);
		gl_utils::renderQuadWithNormal(pts[1], pts[5], pts[7], pts[3]);
		gl_utils::renderQuadWithNormal(pts[1], pts[5], pts[7], pts[3]);
		gl_utils::renderQuadWithNormal(pts[4], pts[5], pts[1], pts[0]);

		glEnd();
	}

	glDisable(GL_BLEND);

#endif
}

// Ctors
CFrustum::CFrustum()
	: m_fov_horz_left(mrpt::DEG2RAD(45)),
	  m_fov_horz_right(mrpt::DEG2RAD(45)),
	  m_fov_vert_down(mrpt::DEG2RAD(30)),
	  m_fov_vert_up(mrpt::DEG2RAD(30)),

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
	  m_lineWidth(lineWidth),
	  m_planes_color(0xE0, 0x00, 0x00, 0x50)  // RGBA
{
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
	CRenderizableDisplayList::notifyChange();
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
	CRenderizableDisplayList::notifyChange();
}
void CFrustum::setHorzFOV(const float fov_horz_degrees)
{
	m_fov_horz_right = m_fov_horz_left = 0.5f * mrpt::DEG2RAD(fov_horz_degrees);
	keep_min(m_fov_horz_left, DEG2RAD(89.9f));
	keep_max(m_fov_horz_left, 0);
	keep_min(m_fov_horz_right, DEG2RAD(89.9f));
	keep_max(m_fov_horz_right, 0);
	CRenderizableDisplayList::notifyChange();
}
void CFrustum::setVertFOV(const float fov_vert_degrees)
{
	m_fov_vert_down = m_fov_vert_up = 0.5f * mrpt::DEG2RAD(fov_vert_degrees);
	keep_min(m_fov_vert_down, DEG2RAD(89.9f));
	keep_max(m_fov_vert_down, 0);
	keep_min(m_fov_vert_up, DEG2RAD(89.9f));
	keep_max(m_fov_vert_up, 0);
	CRenderizableDisplayList::notifyChange();
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
	CRenderizableDisplayList::notifyChange();
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
	CRenderizableDisplayList::notifyChange();
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
