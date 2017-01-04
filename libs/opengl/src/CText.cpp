/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/opengl/CText.h>
#include <mrpt/utils/CStream.h>
#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE( CText, CRenderizable, mrpt::opengl )

CTextPtr CText::Create(const std::string &str) { return CTextPtr( new CText(str) ); }

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
    glDisable(GL_LIGHTING);

	glColor4ub(m_color.R,m_color.G,m_color.B,m_color.A);
    // Set the "cursor" to the XYZ position:
    glRasterPos3f(0,0,0);//m_x,m_y,m_z);

    // Call the lists for drawing the text:
	renderTextBitmap( m_str.c_str(), GLUT_BITMAP_TIMES_ROMAN_10 );

    glEnable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);
#endif
}

/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void  CText::writeToStream(mrpt::utils::CStream &out,int *version) const
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
void  CText::readFromStream(mrpt::utils::CStream &in,int version)
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
