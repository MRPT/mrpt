/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/opengl/CText3D.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/opengl/gl_utils.h>
#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::utils;
using namespace std;

IMPLEMENTS_SERIALIZABLE( CText3D, CRenderizableDisplayList, mrpt::opengl )

CText3DPtr CText3D::Create(
	const std::string &str,
	const std::string &fontName,
	const double scale,
	const mrpt::opengl::TOpenGLFontStyle text_style,
	const double text_spacing,
	const double text_kerning )
{
	return CText3DPtr( new CText3D(str,fontName,scale,text_style,text_spacing,text_kerning) );
}
/*---------------------------------------------------------------
							Constructor
  ---------------------------------------------------------------*/
CText3D::CText3D(
	const std::string &str,
	const std::string &fontName,
	const double scale ,
	const mrpt::opengl::TOpenGLFontStyle text_style,
	const double text_spacing ,
	const double text_kerning ) :
		m_str ( str ),
		m_fontName ( fontName ),
		m_text_style ( text_style ),
		m_text_spacing ( text_spacing ),
		m_text_kerning ( text_kerning )
{
	this->setScale(scale);
}

/*---------------------------------------------------------------
							Destructor
  ---------------------------------------------------------------*/
CText3D::~CText3D()
{
}

/*---------------------------------------------------------------
							render
  ---------------------------------------------------------------*/
void   CText3D::render_dl() const
{
#if MRPT_HAS_OPENGL_GLUT
	glColor4ub(m_color.R,m_color.G,m_color.B,m_color.A);

	mrpt::opengl::gl_utils::glSetFont(m_fontName);
	mrpt::opengl::gl_utils::glDrawText(
		m_str,
		1.0, // Scale
		m_text_style,
		m_text_spacing, m_text_kerning );

#endif
}

/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void  CText3D::writeToStream(mrpt::utils::CStream &out,int *version) const
{
	if (version)
		*version = 0;
	else
	{
		writeToStreamRender(out);
		out << m_str
			<< m_fontName
			<< (uint32_t)m_text_style
			<< m_text_spacing
			<< m_text_kerning;
	}
}

/*---------------------------------------------------------------
	Implements the reading from a CStream capability of
		CSerializable objects
  ---------------------------------------------------------------*/
void  CText3D::readFromStream(mrpt::utils::CStream &in,int version)
{
	switch(version)
	{
	case 0:
		{
			readFromStreamRender(in);

			uint32_t	i;
			in >> m_str
				>> m_fontName
				>> i
				>> m_text_spacing
				>> m_text_kerning;

			m_text_style = TOpenGLFontStyle(i);

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
	CRenderizableDisplayList::notifyChange();
}

void CText3D::getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const
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
