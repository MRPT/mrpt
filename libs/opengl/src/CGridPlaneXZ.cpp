/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/opengl/CGridPlaneXZ.h>
#include <mrpt/utils/CStream.h>

#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::utils;
using namespace std;

IMPLEMENTS_SERIALIZABLE( CGridPlaneXZ, CRenderizableDisplayList, mrpt::opengl )

CGridPlaneXZPtr CGridPlaneXZ::Create(
	float xMin,
	float xMax,
	float zMin,
	float zMax,
	float y,
	float frequency,
	float lineWidth,
	bool  antiAliasing
	)
{
	return CGridPlaneXZPtr( new CGridPlaneXZ( xMin,xMax, zMin, zMax, y, frequency,lineWidth,antiAliasing ) );
}

/** Constructor */
CGridPlaneXZ::CGridPlaneXZ(
	float xMin,
	float xMax,
	float zMin,
	float zMax,
	float y,
	float frequency,
	float lineWidth,
	bool  antiAliasing
	) :
	m_xMin(xMin),m_xMax(xMax),
	m_zMin(zMin),m_zMax(zMax),
	m_plane_y(y),
	m_frequency(frequency),
	m_lineWidth(lineWidth),
	m_antiAliasing(antiAliasing)
{
}


/*---------------------------------------------------------------
							render
  ---------------------------------------------------------------*/
void   CGridPlaneXZ::render_dl() const
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

	ASSERT_(m_frequency>=0);

	for (float z=m_zMin;z<=m_zMax;z+=m_frequency)
	{
		glVertex3f( m_xMin,m_plane_y,z );
	    glVertex3f( m_xMax,m_plane_y,z );
	}

	for (float x=m_xMin;x<=m_xMax;x+=m_frequency)
	{
		glVertex3f( x,m_plane_y,m_zMin );
	    glVertex3f( x,m_plane_y,m_zMax );
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
void  CGridPlaneXZ::writeToStream(mrpt::utils::CStream &out,int *version) const
{

	if (version)
		*version = 1;
	else
	{
		writeToStreamRender(out);
		out << m_xMin << m_xMax;
		out << m_zMin << m_zMax << m_plane_y;
		out << m_frequency;
		out << m_lineWidth << m_antiAliasing; // v1
	}
}

/*---------------------------------------------------------------
	Implements the reading from a CStream capability of
		CSerializable objects
  ---------------------------------------------------------------*/
void  CGridPlaneXZ::readFromStream(mrpt::utils::CStream &in,int version)
{

	switch(version)
	{
	case 0:
	case 1:
		{
			readFromStreamRender(in);
			in >> m_xMin >> m_xMax;
			in >> m_zMin >> m_zMax >> m_plane_y;
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

void CGridPlaneXZ::getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const
{
	bb_min.x = m_xMin;
	bb_min.y = 0;
	bb_min.z = m_zMin;

	bb_max.x = m_xMax;
	bb_max.y = 0;
	bb_max.z = m_zMax;

	// Convert to coordinates of my parent:
	m_pose.composePoint(bb_min, bb_min);
	m_pose.composePoint(bb_max, bb_max);
}
