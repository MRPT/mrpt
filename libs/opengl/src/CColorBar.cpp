/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/opengl/CColorBar.h>
#include <mrpt/serialization/CArchive.h>
#include "gltext.h"

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

void CColorBar::render(const RenderContext& rc) const
{
	// colobars are typically displayed on-top
	switch (rc.shader_id)
	{
		case DefaultShaderID::TRIANGLES:
			CRenderizableShaderTriangles::render(rc);
			break;
		case DefaultShaderID::WIREFRAME:
			CRenderizableShaderWireFrame::render(rc);
			break;
	};
}
void CColorBar::renderUpdateBuffers() const
{
	const_cast<CColorBar&>(*this).onUpdateBuffers_all();

	CRenderizableShaderTriangles::renderUpdateBuffers();
	CRenderizableShaderWireFrame::renderUpdateBuffers();
}

void CColorBar::onUpdateBuffers_all()
{
	auto& tris = CRenderizableShaderTriangles::m_triangles;
	auto& lines_vbd = CRenderizableShaderWireFrame::m_vertex_buffer_data;
	auto& lines_cbd = CRenderizableShaderWireFrame::m_color_buffer_data;
	lines_vbd.clear();
	lines_cbd.clear();

	// precomputed params:
	unsigned int num_divisions = 64;
	unsigned int num_labels = 4;
	unsigned int one_label_each_nth = num_divisions / num_labels;

	const double x0 = .0, x1 = m_width, x2 = m_width * 1.3;
	const double Ay = m_height / (num_divisions - 1);

	std::vector<mrpt::img::TColor> colors(num_divisions);
	for (unsigned int i = 0; i < num_divisions; i++)
	{
		const float col_idx =
			m_min_col + i * (m_max_col - m_min_col) / (num_divisions - 1);
		mrpt::img::TColorf colf;
		mrpt::img::colormap(m_colormap, col_idx, colf.R, colf.G, colf.B);
		colors[i] = colf.asTColor();
	}

	// Text labels:
	mrpt::opengl::internal::glSetFont("mono");

	const auto tickColor = mrpt::img::TColor::black();
	const auto textColor = mrpt::img::TColor::white();

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
			lines_vbd.emplace_back(x0, y0, 0);
			lines_vbd.emplace_back(x2, y0, 0);
			lines_cbd.emplace_back(tickColor);
			lines_cbd.emplace_back(tickColor);

			// Text:

			// Size: m_label_font_size
			// glTranslated(x2, y0, 0.0);
			mrpt::poses::CPose3D p(x2, y0, 0.0, 0, 0, 0);

			mrpt::opengl::internal::glDrawTextTransformed(
				mrpt::format(m_label_format.c_str(), val), tris, lines_vbd,
				lines_cbd, p, m_label_font_size, textColor, FILL, 1.5, 0.1);
		}
	}

	// Color bar itself:
	for (unsigned int i = 0; i < num_divisions - 1; i++)
	{
		const double y0 = Ay * i, y1 = Ay * (i + 1);
		const TPoint3Df pt00(x0, y0, 0), pt10(x1, y0, 0);
		const TPoint3Df pt01(x0, y1, 0), pt11(x1, y1, 0);

		// Color quad:
		// triangle 1/2
		mrpt::opengl::TTriangle t;
		t.vertices[0].xyzrgba.pt = pt00;
		t.vertices[0].setColor(colors[i]);

		t.vertices[1].xyzrgba.pt = pt10;
		t.vertices[1].setColor(colors[i]);

		t.vertices[2].xyzrgba.pt = pt11;
		t.vertices[2].setColor(colors[i + 1]);
		t.computeNormals();
		tris.emplace_back(t);

		// triangle 2/2
		t.vertices[0].xyzrgba.pt = pt00;
		t.vertices[0].setColor(colors[i]);

		t.vertices[1].xyzrgba.pt = pt11;
		t.vertices[1].setColor(colors[i + 1]);

		t.vertices[2].xyzrgba.pt = pt01;
		t.vertices[2].setColor(colors[i + 1]);
		t.computeNormals();
		tris.emplace_back(t);
	}
}

void CColorBar::onUpdateBuffers_Wireframe()
{
	// Already done in onUpdateBuffers_all()
}
void CColorBar::onUpdateBuffers_Triangles()
{
	// Already done in onUpdateBuffers_all()
}

uint8_t CColorBar::serializeGetVersion() const { return 1; }
void CColorBar::serializeTo(mrpt::serialization::CArchive& out) const
{
	writeToStreamRender(out);
	// version 0
	out << uint32_t(m_colormap) << m_min_col << m_max_col << m_min_value
		<< m_max_value << m_label_format << m_label_font_size;
}
void CColorBar::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		case 1:
			readFromStreamRender(in);

			in.ReadAsAndCastTo<uint32_t, mrpt::img::TColormap>(m_colormap);
			in >> m_min_col >> m_max_col >> m_min_value >> m_max_value >>
				m_label_format >> m_label_font_size;
			if (version == 0)
			{
				bool old_disable_depth_test;
				in >> old_disable_depth_test;
			}
			break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
	CRenderizable::notifyChange();
}

auto CColorBar::getBoundingBox() const -> mrpt::math::TBoundingBox
{
	return mrpt::math::TBoundingBox({0, 0, 0}, {m_width, m_height, .0})
		.compose(m_pose);
}
