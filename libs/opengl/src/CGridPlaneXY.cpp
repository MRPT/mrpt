/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/utils/CStream.h>

#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::utils;
using namespace std;

IMPLEMENTS_SERIALIZABLE( CGridPlaneXY, CRenderizableDisplayList, mrpt::opengl )

CGridPlaneXYPtr CGridPlaneXY::Create(
				float xMin,
				float xMax,
				float yMin,
				float yMax,
				float z,
				float frequency, 
				float lineWidth,
				bool  antiAliasing)
{
	return CGridPlaneXYPtr( new CGridPlaneXY(xMin,xMax,yMin,yMax, z, frequency,lineWidth,antiAliasing ) );
}
/** Constructor  */
CGridPlaneXY::CGridPlaneXY(
	float xMin,
	float xMax,
	float yMin,
	float yMax,
	float z,
	float frequency,
	float lineWidth,
	bool  antiAliasing
	) :
	m_xMin(xMin),
	m_xMax(xMax),
	m_yMin(yMin),
	m_yMax(yMax),
	m_plane_z(z),
	m_frequency(frequency),
	m_lineWidth(lineWidth),
	m_antiAliasing(antiAliasing)
{
}

/*---------------------------------------------------------------
					render_dl
  ---------------------------------------------------------------*/
void   CGridPlaneXY::render_dl() const
{
#if MRPT_HAS_OPENGL_GLUT
	ASSERT_(m_frequency>=0)

	// Enable antialiasing:
	if (m_antiAliasing)
	{
		glPushAttrib( GL_COLOR_BUFFER_BIT | GL_LINE_BIT );
		glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
		glEnable(GL_BLEND);
		glEnable(GL_LINE_SMOOTH);
	}
	glLineWidth(m_lineWidth);

	glDisable(GL_LIGHTING);  // Disable lights when drawing lines
	glBegin(GL_LINES);

	for (float y=m_yMin;y<=m_yMax;y+=m_frequency)
	{
		glVertex3f( m_xMin,y,m_plane_z );
		glVertex3f( m_xMax,y,m_plane_z );
	}

	for (float x=m_xMin;x<=m_xMax;x+=m_frequency)
	{
		glVertex3f( x,m_yMin,m_plane_z );
		glVertex3f( x,m_yMax,m_plane_z );
	}

	glEnd();
	glEnable(GL_LIGHTING);

	// End antialiasing:
	if (m_antiAliasing)
	{
		glPopAttrib();
		checkOpenGLError();
	}
#endif
}

/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void  CGridPlaneXY::writeToStream(mrpt::utils::CStream &out,int *version) const
{

	if (version)
		*version = 1;
	else
	{
		writeToStreamRender(out);
		out << m_xMin << m_xMax;
		out << m_yMin << m_yMax << m_plane_z;
		out << m_frequency;
		out << m_lineWidth << m_antiAliasing; // v1
	}
}

/*---------------------------------------------------------------
	Implements the reading from a CStream capability of
		CSerializable objects
  ---------------------------------------------------------------*/
void  CGridPlaneXY::readFromStream(mrpt::utils::CStream &in,int version)
{

	switch(version)
	{
	case 0:
	case 1:
		{
			readFromStreamRender(in);
			in >> m_xMin >> m_xMax;
			in >> m_yMin >> m_yMax >> m_plane_z;
			in >> m_frequency;
			if (version>=1)
				in >> m_lineWidth >> m_antiAliasing;
			else
			{
				m_lineWidth=1.0f;
				m_antiAliasing=true;
			}

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
	CRenderizableDisplayList::notifyChange();
}

void CGridPlaneXY::getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const
{
	bb_min.x = m_xMin;
	bb_min.y = m_yMin;
	bb_min.z = 0;

	bb_max.x = m_xMax;
	bb_max.y = m_yMax;
	bb_max.z = 0;

	// Convert to coordinates of my parent:
	m_pose.composePoint(bb_min, bb_min);
	m_pose.composePoint(bb_max, bb_max);
}
