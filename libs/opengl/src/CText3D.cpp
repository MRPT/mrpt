/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/opengl.h>  // Precompiled header


#include <mrpt/opengl/CText3D.h>
#include <mrpt/opengl/gl_utils.h>
#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::utils;
using namespace std;

IMPLEMENTS_SERIALIZABLE( CText3D, CRenderizableDisplayList, mrpt::opengl )

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
void  CText3D::writeToStream(CStream &out,int *version) const
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
void  CText3D::readFromStream(CStream &in,int version)
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
}
