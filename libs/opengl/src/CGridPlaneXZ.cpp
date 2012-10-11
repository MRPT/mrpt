/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
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


#include <mrpt/opengl/CGridPlaneXZ.h>

#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::utils;
using namespace std;

IMPLEMENTS_SERIALIZABLE( CGridPlaneXZ, CRenderizableDisplayList, mrpt::opengl )

/*---------------------------------------------------------------
							render
  ---------------------------------------------------------------*/
void   CGridPlaneXZ::render_dl() const
{
#if MRPT_HAS_OPENGL_GLUT
	MRPT_START

	glLineWidth(1);

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
	checkOpenGLError();

	MRPT_END
#endif
}

/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void  CGridPlaneXZ::writeToStream(CStream &out,int *version) const
{

	if (version)
		*version = 0;
	else
	{
		writeToStreamRender(out);
		out << m_xMin << m_xMax;
		out << m_zMin << m_zMax << m_plane_y;
		out << m_frequency;
	}
}

/*---------------------------------------------------------------
	Implements the reading from a CStream capability of
		CSerializable objects
  ---------------------------------------------------------------*/
void  CGridPlaneXZ::readFromStream(CStream &in,int version)
{

	switch(version)
	{
	case 0:
		{
			readFromStreamRender(in);
			in >> m_xMin >> m_xMax;
			in >> m_zMin >> m_zMax >> m_plane_y;
			in >> m_frequency;
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
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
