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


#include <mrpt/opengl/CArrow.h>
#include <mrpt/math/CMatrix.h>
#include <mrpt/math/geometry.h>

#include "opengl_internals.h"


using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;


IMPLEMENTS_SERIALIZABLE( CArrow, CRenderizableDisplayList, mrpt::opengl )


/*---------------------------------------------------------------
							render
  ---------------------------------------------------------------*/
void   CArrow::render_dl() const
{
#if MRPT_HAS_OPENGL_GLUT
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
    glShadeModel(GL_SMOOTH);


	GLUquadricObj	*obj1 = gluNewQuadric();
	GLUquadricObj	*obj2 = gluNewQuadric();

    GLfloat		mat[16];

    // Compute the direction vector, which will become the transformed z-axis:
    float		vx = m_x1-m_x0;
    float		vy = m_y1-m_y0;
    float		vz = m_z1-m_z0;
	if ((m_arrow_roll!=-1.0f)||(m_arrow_pitch!=-1.0f)||(m_arrow_yaw!=-1.0f))
	{
		m_x0 = 0.0f;
		m_x1 = 0.0f;
		m_y0 = 0.0f;
		m_y1 = 0.1f;
		m_z0 = 0.0f;
		m_z1 = 0.0f;

		float cr = cos(m_arrow_roll);
		float sr = sin(m_arrow_roll);
		float cp = cos(m_arrow_pitch);
		float sp = sin(m_arrow_pitch);
		float cy = cos(m_arrow_yaw);
		float sy = sin(m_arrow_yaw);

		CMatrixFloat m(3,3),xx(3,1),out(1,3);
		m(0,0) = cr*cp;			m(0,1) = cr*sp*sy - sr*cy;			m(0,2) = sr*sy + cr*sp*cy;
		m(1,0) = sr*cp;			m(1,1) = sr*sp*sy + cr*cy;			m(1,2) = sr*sp*cy - cr*sy;
		m(2,0) = -sp;			m(2,1) = cp*sy;						m(2,2) = cp*cy;
		xx(0,0) = 0.0f;
		xx(1,0) = 1.0f;
		xx(2,0) = 0.0f;

		out = m * xx;
		vx = out(0,0);
		vy = out(1,0);
		vz = out(2,0);
	}

    // Normalize:
    float		v_mod = sqrt( square(vx)+square(vy)+square(vz) );
    if (v_mod>0)
    {
    	vx/=v_mod;
    	vy/=v_mod;
    	vz/=v_mod;
    }

    //  A homogeneous transformation matrix, in this order:
    //
    //     0  4  8  12
    //     1  5  9  13
    //     2  6  10 14
    //     3  7  11 15
    //

    mat[3] = mat[7] = mat[11] = 0;
    mat[15] = 1;
    mat[12] = m_x0; mat[13] = m_y0; mat[14] = m_z0;

    // New Z-axis
	mat[8] = vx;
	mat[9] = vy;
	mat[10] = vz;

    // New X-axis: Perp. to Z
    if (vx!=0 || vy!=0)
    {
		mat[0] = -vy;
		mat[1] = vx;
		mat[2] = 0;
    }
    else
    {
		mat[0] = 0;
		mat[1] = vz;
		mat[2] = -vy;
    }

    // New Y-axis: Perp. to both: the cross product:
    //  | i  j  k |     | i  j  k |
    //  | x0 y0 z0| --> | 8  9  10|
    //  | x1 y1 z1|     | 0  1  2 |
    GLfloat *out_v3 = mat+4;
	math::crossProduct3D(
		mat+8, // 1st vector
		mat+0, // 2nd vector
		out_v3  // Output cross product
		);

    glPushMatrix();

    glMultMatrixf( mat );
	// Scale Z to the size of the cylinder:
    glScalef(1.0f,1.0f,1.0f-m_headRatio);
	gluCylinder( obj1, m_smallRadius, m_smallRadius, 1, 10, 1 );

    glPopMatrix();

    // Draw the head of the arrow: a cone (built from a cylinder)
	//-------------------------------------------------------------
    mat[12] = m_x0 + vx*(1.0f-m_headRatio);
	mat[13] = m_y0 + vy*(1.0f-m_headRatio);
	mat[14] = m_z0 + vz*(1.0f-m_headRatio);

    glPushMatrix();

    glMultMatrixf( mat );
	// Scale Z to the size of the cylinder:
    glScalef(1.0f,1.0f,m_headRatio);

	gluCylinder( obj2, m_largeRadius, 0, 1, 10, 10 );

    glPopMatrix();

	gluDeleteQuadric(obj1);
	gluDeleteQuadric(obj2);

	glDisable(GL_LIGHTING);
	glDisable(GL_LIGHT0);
    glDisable(GL_COLOR_MATERIAL);
#endif
}

/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void  CArrow::writeToStream(CStream &out,int *version) const
{
	if (version)
		*version = 1;
	else
	{
		writeToStreamRender(out);
		out << m_x0 << m_y0 << m_z0;
		out << m_x1 << m_y1 << m_z1;
		out << m_headRatio << m_smallRadius << m_largeRadius;
		out << m_arrow_roll << m_arrow_pitch << m_arrow_yaw;

	}
}

/*---------------------------------------------------------------
	Implements the reading from a CStream capability of
		CSerializable objects
  ---------------------------------------------------------------*/
void  CArrow::readFromStream(CStream &in,int version)
{
	switch(version)
	{
	case 0:
		{
			readFromStreamRender(in);
			in >> m_x0 >> m_y0 >> m_z0;
			in >> m_x1 >> m_y1 >> m_z1;
			in >> m_headRatio >> m_smallRadius >> m_largeRadius;
		}
        break;
	case 1:
		{
			readFromStreamRender(in);
			in >> m_x0 >> m_y0 >> m_z0;
			in >> m_x1 >> m_y1 >> m_z1;
			in >> m_headRatio >> m_smallRadius >> m_largeRadius;
			in >> m_arrow_roll >> m_arrow_pitch >> m_arrow_yaw;
		}
		break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
}

