/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/opengl/CDisk.h>
#include <mrpt/utils/CStream.h>

#include "opengl_internals.h"


using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::utils;
using mrpt::poses::CPose3D;
using namespace std;



IMPLEMENTS_SERIALIZABLE( CDisk, CRenderizableDisplayList, mrpt::opengl )

CDiskPtr CDisk::Create(float radiusOut,float radiusIn,uint32_t slices,uint32_t loops)	
{
	return CDiskPtr(new CDisk(radiusOut,radiusIn,slices,loops));
}
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
void  CDisk::writeToStream(mrpt::utils::CStream &out,int *version) const
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
void  CDisk::readFromStream(mrpt::utils::CStream &in,int version)
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
	CRenderizableDisplayList::notifyChange();
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

void CDisk::getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const
{
	bb_min.x = -std::max(m_radiusIn, m_radiusOut);
	bb_min.y = bb_min.x;
	bb_min.z = 0;

	bb_max.x = std::max(m_radiusIn, m_radiusOut);
	bb_max.y = bb_max.x;
	bb_max.z = 0;

	// Convert to coordinates of my parent:
	m_pose.composePoint(bb_min, bb_min);
	m_pose.composePoint(bb_max, bb_max);
}
