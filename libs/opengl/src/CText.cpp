/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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

void CText::getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const
{
	bb_min.x = 0;
	bb_min.y = 0;
	bb_min.z = 0;

	bb_max = bb_min;

	// Convert to coordinates of my parent:
	m_pose.composePoint(bb_min, bb_min);
	m_pose.composePoint(bb_max, bb_max);
}
