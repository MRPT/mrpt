/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/opengl/CText.h>
#include <mrpt/serialization/CArchive.h>

#include "gltext.h"
#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CText, CRenderizable, mrpt::opengl)

CText::~CText() = default;

void CText::onUpdateBuffers_Text()
{
	const double text_spacing = 1.5;
	const double text_kerning = 0.1;

	mrpt::opengl::internal::glSetFont(m_fontName);
	mrpt::opengl::internal::glDrawText(
		m_str, CRenderizableShaderText::m_triangles,
		CRenderizableShaderText::m_vertex_buffer_data, mrpt::opengl::FILL,
		text_spacing, text_kerning);

	// All lines & triangles, the same color:
	CRenderizableShaderText::m_color_buffer_data.assign(
		m_vertex_buffer_data.size(), m_color);
	for (auto& tri : m_triangles) tri.setColor(m_color);
}

void CText::render(const RenderContext& rc) const
{
#if MRPT_HAS_OPENGL_GLUT

	// Compute pixel of (0,0,0) for this object:
	// "px" will be given in the range:
	//     [-1,-1] (left-bottom) - [+1,+1] (top-right)
	//
	const auto& pmv = rc.state->pmv_matrix;
	const auto px =
		mrpt::math::TPoint2Df(pmv(0, 3) / pmv(3, 3), pmv(1, 3) / pmv(3, 3));

	// Load matrices in shader:
	const GLint u_pmat = rc.shader->uniformId("p_matrix");
	const GLint u_mvmat = rc.shader->uniformId("mv_matrix");

	// Projection: identity
	static const auto eye4 = mrpt::math::CMatrixFloat44::Identity();
	glUniformMatrix4fv(u_pmat, 1, true, eye4.data());

	// Model-view: translate and scale
	auto mv = mrpt::math::CMatrixFloat44::Identity();
	mv(0, 3) = px.x;
	mv(1, 3) = px.y;
	mv(2, 3) = pmv(2, 3) / pmv(3, 3);  // depth

	if (rc.state->viewport_height <= 0 || rc.state->viewport_width <= 0)
	{
		std::cerr << "[CText] Warning: invalid viewport size!\n";
		return;
	}

	// Find scale according to font height in pixels:
	const float scale =
		this->m_fontHeight / static_cast<float>(rc.state->viewport_height);

	const float aspect =
		rc.state->viewport_width / double(rc.state->viewport_height);
	mv(0, 0) *= scale / aspect;
	mv(1, 1) *= scale;

	glUniformMatrix4fv(u_mvmat, 1, true, mv.data());

	CRenderizableShaderText::render(rc);

#endif
}

uint8_t CText::serializeGetVersion() const { return 1; }
void CText::serializeTo(mrpt::serialization::CArchive& out) const
{
	writeToStreamRender(out);
	out << m_str;
	out << m_fontName;
	out << (uint32_t)m_fontHeight << (uint32_t)m_fontWidth;
}

void CText::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		case 1:
		{
			uint32_t i;
			readFromStreamRender(in);
			in >> m_str;
			if (version >= 1)
			{
				in >> m_fontName;
				in >> i;
				m_fontHeight = i;
				in >> i;
				m_fontWidth = i;
			}
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
}

void CText::getBoundingBox(
	mrpt::math::TPoint3D& bb_min, mrpt::math::TPoint3D& bb_max) const
{
	bb_min.x = 0;
	bb_min.y = 0;
	bb_min.z = 0;

	bb_max = bb_min;

	// Convert to coordinates of my parent:
	m_pose.composePoint(bb_min, bb_min);
	m_pose.composePoint(bb_max, bb_max);
}
