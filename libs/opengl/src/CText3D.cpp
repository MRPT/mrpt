/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/opengl/CText3D.h>
#include <mrpt/opengl/gl_utils.h>
#include <mrpt/serialization/CArchive.h>
#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::opengl;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CText3D, CRenderizable, mrpt::opengl)

/*---------------------------------------------------------------
							Constructor
  ---------------------------------------------------------------*/
CText3D::CText3D(
	const std::string& str, const std::string& fontName, const double scale,
	const mrpt::opengl::TOpenGLFontStyle text_style, const double text_spacing,
	const double text_kerning)
	: m_str(str),
	  m_fontName(fontName),
	  m_text_style(text_style),
	  m_text_spacing(text_spacing),
	  m_text_kerning(text_kerning)
{
	this->setScale(scale);
}

CText3D::~CText3D() = default;

void CText3D::renderUpdateBuffers() const
{
	//
	MRPT_TODO("Implement me!");
}

void CText3D::render() const
{
#if MRPT_HAS_OPENGL_GLUT
	glColor4ub(m_color.R, m_color.G, m_color.B, m_color.A);

	mrpt::opengl::gl_utils::glSetFont(m_fontName);
	mrpt::opengl::gl_utils::glDrawText(
		m_str,
		1.0,  // Scale
		m_text_style, m_text_spacing, m_text_kerning);

#endif
}

uint8_t CText3D::serializeGetVersion() const { return 0; }
void CText3D::serializeTo(mrpt::serialization::CArchive& out) const
{
	writeToStreamRender(out);
	out << m_str << m_fontName << (uint32_t)m_text_style << m_text_spacing
		<< m_text_kerning;
}

void CText3D::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		{
			readFromStreamRender(in);

			uint32_t i;
			in >> m_str >> m_fontName >> i >> m_text_spacing >> m_text_kerning;

			m_text_style = TOpenGLFontStyle(i);
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
	CRenderizable::notifyChange();
}

void CText3D::getBoundingBox(
	mrpt::math::TPoint3D& bb_min, mrpt::math::TPoint3D& bb_max) const
{
	bb_min.x = 0;
	bb_min.y = 0;
	bb_min.z = 0;

	bb_max.x = m_str.size() * m_scale_x;
	bb_max.y = 1;
	bb_max.z = 0;

	// Convert to coordinates of my parent:
	m_pose.composePoint(bb_min, bb_min);
	m_pose.composePoint(bb_max, bb_max);
}
