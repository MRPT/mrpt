/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/opengl/CColorBar.h>
#include <mrpt/opengl/gl_utils.h>
#include <mrpt/serialization/CArchive.h>

#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CColorBar, CRenderizable, mrpt::opengl)

CColorBar::CColorBar(
	/** The colormap to represent. */
	const mrpt::img::TColormap colormap,
	/** size of the color bar */
	double width, double height,
	/** limits for [0,1] colormap indices */
	float min_col, float max_col,
	/** limits for values associated to extreme colors */
	float min_value, float max_value,
	/** sprintf-like format string for values */
	const std::string& label_format,
	/** Label text font size */
	float label_font_size)
	: m_colormap(colormap),
	  m_width(width),
	  m_height(height),
	  m_label_format(label_format),
	  m_min_col(min_col),
	  m_max_col(max_col),
	  m_min_value(min_value),
	  m_max_value(max_value),
	  m_label_font_size(label_font_size)

{
}

void CColorBar::setColormap(const mrpt::img::TColormap colormap)
{
	m_colormap = colormap;
	CRenderizable::notifyChange();
}

void CColorBar::setColorAndValueLimits(
	float col_min, float col_max, float value_min, float value_max)
{
	m_min_col = col_min;
	m_max_col = col_max;
	m_min_value = value_min;
	m_max_value = value_max;
	CRenderizable::notifyChange();
}

void mrpt::opengl::CColorBar::enableDepthTest(bool enable)
{
	m_disable_depth_test = enable;
	CRenderizable::notifyChange();
}

void CColorBar::renderUpdateBuffers() const
{
	//
	MRPT_TODO("Implement me!");
}
void CColorBar::render(const RenderContext& rc) const
{
#if 0 && MRPT_HAS_OPENGL_GLUT
	if (m_disable_depth_test)
		glDisable(GL_DEPTH_TEST);  // colobars are typically displayed on-top of
	// the rest of objects!
	glDisable(GL_LIGHTING);

	// solid:
	unsigned int num_divisions = 64;
	unsigned int num_labels = 4;
	unsigned int one_label_each_nth = num_divisions / num_labels;

	const double x0 = .0, x1 = m_width, x2 = m_width * 1.3;
	const double Ay = m_height / (num_divisions - 1);

	std::vector<mrpt::img::TColorf> cols(num_divisions);
	for (unsigned int i = 0; i < num_divisions; i++)
	{
		const float col_idx =
			m_min_col + i * (m_max_col - m_min_col) / (num_divisions - 1);
		mrpt::img::colormap(
			m_colormap, col_idx, cols[i].R, cols[i].G, cols[i].B);
	}

	for (unsigned int i = 0; i < num_divisions - 1; i++)
	{
		const double y0 = Ay * i, y1 = Ay * (i + 1);
		const TPoint3D pt00(x0, y0, 0), pt10(x1, y0, 0);
		const TPoint3D pt01(x0, y1, 0), pt11(x1, y1, 0);

		// Color quad:

		glBegin(GL_TRIANGLES);
		glColor3f(cols[i].R, cols[i].G, cols[i].B);
		glVertex3d(pt00.x, pt00.y, pt00.z);
		glVertex3d(pt10.x, pt10.y, pt10.z);
		glColor3f(cols[i + 1].R, cols[i + 1].G, cols[i + 1].B);
		glVertex3d(pt11.x, pt11.y, pt11.z);
		//
		glColor3f(cols[i].R, cols[i].G, cols[i].B);
		glVertex3d(pt00.x, pt00.y, pt00.z);
		glColor3f(cols[i + 1].R, cols[i + 1].G, cols[i + 1].B);
		glVertex3d(pt11.x, pt11.y, pt11.z);
		glVertex3d(pt01.x, pt01.y, pt01.z);
		glEnd();
	}

	mrpt::opengl::gl_utils::glSetFont("mono");

	for (unsigned int i = 0; i < num_divisions; i++)
	{
		const double val =
			m_min_value + i * (m_max_value - m_min_value) / (num_divisions - 1);
		const double y0 = Ay * i;  //, y1 = Ay*(i + 1);

		// Text label:
		bool draw_label =
			(i % one_label_each_nth) == 0 || i == (num_divisions - 1);

		if (draw_label)
		{
			// Line:
			glLineWidth(1.0);
			glBegin(GL_LINES);
			glColor3b(0, 0, 0);
			glVertex2d(x0, y0);
			glVertex2d(x2, y0);
			glEnd();

			// Text:
			glPushMatrix();

			glTranslated(x2, y0, 0.0);
			glColor3ub(0xff, 0xff, 0xff);
			mrpt::opengl::gl_utils::glDrawText(
				mrpt::format(m_label_format.c_str(), val), m_label_font_size);

			glPopMatrix();
		}
	}

#endif
}

uint8_t CColorBar::serializeGetVersion() const { return 0; }
void CColorBar::serializeTo(mrpt::serialization::CArchive& out) const
{
	writeToStreamRender(out);
	// version 0
	out << uint32_t(m_colormap) << m_min_col << m_max_col << m_min_value
		<< m_max_value << m_label_format << m_label_font_size
		<< m_disable_depth_test;
}
void CColorBar::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
			readFromStreamRender(in);

			in.ReadAsAndCastTo<uint32_t, mrpt::img::TColormap>(m_colormap);
			in >> m_min_col >> m_max_col >> m_min_value >> m_max_value >>
				m_label_format >> m_label_font_size >> m_disable_depth_test;
			break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
	CRenderizable::notifyChange();
}

void CColorBar::getBoundingBox(
	mrpt::math::TPoint3D& bb_min, mrpt::math::TPoint3D& bb_max) const
{
	bb_min.x = 0;
	bb_min.y = 0;
	bb_min.z = 0;

	bb_max.x = m_width;
	bb_max.y = m_height;
	bb_max.z = 0;

	// Convert to coordinates of my parent:
	m_pose.composePoint(bb_min, bb_min);
	m_pose.composePoint(bb_max, bb_max);
}
