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


#include <mrpt/opengl/CText.h>
#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE( CText, CRenderizable, mrpt::opengl )

/*---------------------------------------------------------------
							Constructor
  ---------------------------------------------------------------*/
CText::CText( const string &str )
{
	m_str 	= str;

    m_fontName = "Arial";
    m_fontHeight = 10;
    m_fontWidth = 0;
}

/*---------------------------------------------------------------
							Destructor
  ---------------------------------------------------------------*/
CText::~CText()
{
}

/*---------------------------------------------------------------
							render
  ---------------------------------------------------------------*/
void   CText::render() const
{
#if MRPT_HAS_OPENGL_GLUT
    glDisable(GL_DEPTH_TEST);

	glColor4ub(m_color.R,m_color.G,m_color.B,m_color.A);
    // Set the "cursor" to the XYZ position:
    glRasterPos3f(0,0,0);//m_x,m_y,m_z);

    // Call the lists for drawing the text:
	renderTextBitmap( m_str.c_str(), GLUT_BITMAP_TIMES_ROMAN_10 );

    glEnable(GL_DEPTH_TEST);
#endif
}

/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void  CText::writeToStream(CStream &out,int *version) const
{
	if (version)
		*version = 1;
	else
	{
		writeToStreamRender(out);
		out << m_str;
        out << m_fontName;
        out << (uint32_t)m_fontHeight << (uint32_t)m_fontWidth;

	}
}

/*---------------------------------------------------------------
	Implements the reading from a CStream capability of
		CSerializable objects
  ---------------------------------------------------------------*/
void  CText::readFromStream(CStream &in,int version)
{
	switch(version)
	{
	case 0:
	case 1:
		{
			uint32_t	i;
			readFromStreamRender(in);
			in >> m_str;
			if (version>=1)
			{
				in >> m_fontName;
				in >> i; m_fontHeight = i;
				in >> i; m_fontWidth = i;
			}
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
}
