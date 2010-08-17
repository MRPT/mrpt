/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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


#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/math/utils.h>

#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE( CPointCloudColoured, CRenderizable, mrpt::opengl )

/*---------------------------------------------------------------
							render
  ---------------------------------------------------------------*/
void   CPointCloudColoured::render() const
{
#if MRPT_HAS_OPENGL_GLUT

	if ( m_color_A != 1.0 )
	{
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	}

    glPointSize( m_pointSize );

    if (m_pointSmooth)
			glEnable ( GL_POINT_SMOOTH );
	else 	glDisable( GL_POINT_SMOOTH );

    glBegin( GL_POINTS );

    glColor4f( m_color_R,m_color_G,m_color_B,m_color_A );

    for (const_iterator i=begin();i!=end();++i)
    {
		glColor4f( i->R,i->G,i->B,m_color_A );
		glVertex3f( i->x,i->y,i->z );
    }

    glEnd();

	// Undo flags:
	if ( m_color_A != 1.0 )
		glDisable(GL_BLEND);

	if (m_pointSmooth)
		glDisable( GL_POINT_SMOOTH );

	checkOpenGLError();
#endif
}

/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void  CPointCloudColoured::writeToStream(CStream &out,int *version) const
{

	if (version)
		*version = 2;
	else
	{
		writeToStreamRender(out);
		out << m_points;
		out << m_pointSize;
		out << m_pointSmooth; // Added in v2
	}
}

/*---------------------------------------------------------------
	Implements the reading from a CStream capability of
		CSerializable objects
  ---------------------------------------------------------------*/
void  CPointCloudColoured::readFromStream(CStream &in,int version)
{
	switch(version)
	{
	case 1:
	case 2:
		{
			readFromStreamRender(in);
			in >> m_points >> m_pointSize;

			if (version>=2)
					in >> m_pointSmooth;
			else 	m_pointSmooth = false;

		} break;
	case 0:
		{
			readFromStreamRender(in);

			// Old vector_serializable:
			uint32_t n;
			in >> n;
			m_points.resize(n);
			for (uint32_t i=0;i<n;i++) in >> m_points[i];

			in >> m_pointSize;
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

CStream& mrpt::opengl::operator >> (CStream& in,  CPointCloudColoured::TPointColour &o)
{
	in >> o.x >> o.y >> o.z >> o.R >> o.G >> o.B;
	return in;
}


CStream& mrpt::opengl::operator << (CStream& out, const CPointCloudColoured::TPointColour &o)
{
	out << o.x << o.y << o.z << o.R << o.G << o.B;
	return out;
}
