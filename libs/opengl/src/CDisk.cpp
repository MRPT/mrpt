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

#include <mrpt/opengl/CDisk.h>

#include "opengl_internals.h"


using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::utils;
using mrpt::poses::CPose3D;
using namespace std;



IMPLEMENTS_SERIALIZABLE( CDisk, CRenderizableDisplayList, mrpt::opengl )

/*---------------------------------------------------------------
							render
  ---------------------------------------------------------------*/
void   CDisk::render_dl() const
{
#if MRPT_HAS_OPENGL_GLUT
	glEnable (GL_BLEND);
	checkOpenGLError();
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	checkOpenGLError();

	GLUquadricObj	*obj = gluNewQuadric();

	gluDisk(
		obj,
		m_radiusIn,
		m_radiusOut,
		m_nSlices,
		m_nLoops );

	gluDeleteQuadric(obj);

	glDisable(GL_BLEND);
#endif
}

/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void  CDisk::writeToStream(CStream &out,int *version) const
{

	if (version)
		*version = 0;
	else
	{
		writeToStreamRender(out);
		out << m_radiusIn << m_radiusOut;
		out << m_nSlices << m_nLoops;
	}
}

/*---------------------------------------------------------------
	Implements the reading from a CStream capability of
		CSerializable objects
  ---------------------------------------------------------------*/
void  CDisk::readFromStream(CStream &in,int version)
{
	switch(version)
	{
	case 0:
		{
			readFromStreamRender(in);
			in >> m_radiusIn >> m_radiusOut;
			in >> m_nSlices;
			in >> m_nLoops;
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
}

bool CDisk::traceRay(const mrpt::poses::CPose3D &o,double &dist) const	{
	//The disk is contained initially in a plane which contains (0,0,0), (1,0,0) and (0,1,0)
	//These points are converted into:
	//(x,y,z)
	//( cos(w)*cos(p)+x, sin(w)*cos(p)*y, -sin(p)+z )
	//( -sin(w)*cos(r)+cos(w)*sin(p)*sin(r)+x, cos(w)*cos(r)+sin(w)*sin(p)*sin(r)+y, cos(p)*sin(r)*z )
	CPose3D transf=this->m_pose-o;
	double x=transf.x(),y=transf.y(),z=transf.z(),w=transf.yaw(),p=transf.pitch(),r=transf.roll();
	double coef=sin(w)*sin(r)+cos(w)*sin(p)*cos(r);
	//coef is the first component of the normal to the transformed Z plane. So, the scalar product between
	//this normal and (1,0,0) (which happens to be the beam's vector) equals coef. And if it's 0, then both
	//are orthogonal, that is, the beam is parallel to the plane.
	if (coef==0) return false;
	//The following expression yields the collision point between the plane and the beam (the y and z
	//coordinates are zero).
	dist=x+(y*(sin(p)*sin(w)*cos(r)-cos(w)*sin(r))+z*cos(p)*cos(r))/coef;
	if (dist<0) return false;
	//Euclidean distance is invariant to rotations...
	double d2=(x-dist)*(x-dist)+y*y+z*z;
	return d2>=(m_radiusIn*m_radiusIn)&&d2<=(m_radiusOut*m_radiusOut);

	//IMPORTANT NOTICE: using geometric intersection between Z plane and CPose's line intersection is SLOWER than the used method.
}

