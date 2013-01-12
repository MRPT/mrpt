/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
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


#include <mrpt/opengl/CSphere.h>
#include <mrpt/poses/CPose3D.h>
#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::poses;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;


IMPLEMENTS_SERIALIZABLE( CSphere, CRenderizableDisplayList, mrpt::opengl )

/*---------------------------------------------------------------
							render_dl
  ---------------------------------------------------------------*/
void   CSphere::render_dl() const
{
#if MRPT_HAS_OPENGL_GLUT
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
    glShadeModel(GL_SMOOTH);

	if ( m_color.A != 255 )
	{
		glDisable(GL_DEPTH_TEST);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	}

	// Determine radius depending on eye distance?
	float real_radius;
	if (m_keepRadiusIndependentEyeDistance)
	{
		glRasterPos3f(0.0f,0.0f,0.0f);

		GLfloat		raster_pos[4];
		glGetFloatv( GL_CURRENT_RASTER_POSITION, raster_pos);
		float eye_distance= raster_pos[3];

		eye_distance  = max( eye_distance , 0.1f);

		real_radius = 0.01*m_radius * eye_distance;
	}
	else real_radius = m_radius;


	GLUquadricObj	*obj = gluNewQuadric();
	checkOpenGLError();

    gluQuadricDrawStyle(obj,GLU_FILL);
    gluQuadricNormals(obj,GLU_SMOOTH);

	gluSphere( obj, real_radius ,m_nDivsLongitude,m_nDivsLatitude);
	checkOpenGLError();

	gluDeleteQuadric(obj);
	checkOpenGLError();

	if ( m_color.A != 255 )
	{
		glEnable(GL_DEPTH_TEST);
		glDisable(GL_BLEND);
	}

	glDisable(GL_LIGHTING);
	glDisable(GL_LIGHT0);
    glDisable(GL_COLOR_MATERIAL);
#endif
}
/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void  CSphere::writeToStream(CStream &out,int *version) const
{
	if (version)
		*version = 1;
	else
	{
		writeToStreamRender(out);
		out << m_radius;
		out << (uint32_t)m_nDivsLongitude << (uint32_t)m_nDivsLatitude
		    << m_keepRadiusIndependentEyeDistance;
	}
}

/*---------------------------------------------------------------
	Implements the reading from a CStream capability of
		CSerializable objects
  ---------------------------------------------------------------*/
void  CSphere::readFromStream(CStream &in,int version)
{

	switch(version)
	{
	case 0:
	case 1:
		{
			readFromStreamRender(in);
			in >> m_radius;
			uint32_t	i,j;
			in >> i >> j;
			m_nDivsLongitude = i;
			m_nDivsLatitude = j;
			if (version>=1)
					in >> m_keepRadiusIndependentEyeDistance;
			else	m_keepRadiusIndependentEyeDistance = false;
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
	CRenderizableDisplayList::notifyChange();
}

bool CSphere::traceRay(const mrpt::poses::CPose3D &o,double &dist) const	{
	//We need to find the points of the sphere which collide with the laser beam.
	//The sphere's equation is invariant to rotations (but not to translations), and we can take advantage of this;
	//we'll simply transform the center and then compute the beam's points whose distance to that transformed point
	//equals the sphere's radius.

	CPose3D transf=this->m_pose-o;
	double x=transf.x(),y=transf.y(),z=transf.z();
	double r2=m_radius*m_radius;
	double dyz=y*y+z*z;
	if (dyz>r2) return false;
	double dx=sqrt(r2-dyz);
	if (x-dx>=0)	{
		dist=x-dx;
		return true;
	}	else if (x+dx>=0)	{
		dist=x+dx;
		return true;
	}	else return false;
}

void CSphere::getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const
{
	bb_min.x = -m_radius;
	bb_min.y = -m_radius;
	bb_min.z = -m_radius;

	bb_max.x = m_radius;
	bb_max.y = m_radius;
	bb_max.z = m_radius;
	
	// Convert to coordinates of my parent:
	m_pose.composePoint(bb_min, bb_min);
	m_pose.composePoint(bb_max, bb_max);
}
