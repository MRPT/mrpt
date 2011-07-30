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
#include <mrpt/opengl/CCylinder.h>
#include <mrpt/math/geometry.h>

#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::utils;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CCylinder,CRenderizableDisplayList,mrpt::opengl)

/*---------------------------------------------------------------
							render
  ---------------------------------------------------------------*/
void CCylinder::render_dl() const	{
#if MRPT_HAS_OPENGL_GLUT
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
    glShadeModel(GL_SMOOTH);

	glEnable (GL_BLEND);
	checkOpenGLError();
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
	checkOpenGLError();
	GLUquadricObj *obj=gluNewQuadric();
	gluCylinder(obj,mBaseRadius,mTopRadius,mHeight,mSlices,mStacks);
	if (mHasBottomBase) gluDisk(obj,0,mBaseRadius,mSlices,1);
	if (mHasTopBase&&mTopRadius>0)	{
		glPushMatrix();
		glTranslatef(0,0,mHeight);
		gluDisk(obj,0,mTopRadius,mSlices,1);
		glPopMatrix();
	}
	gluDeleteQuadric(obj);
	glDisable(GL_BLEND);

	glDisable(GL_LIGHTING);
	glDisable(GL_LIGHT0);
    glDisable(GL_COLOR_MATERIAL);
#endif
}

/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void CCylinder::writeToStream(CStream &out,int *version) const	{
	if (version) *version=0;
	else	{
		writeToStreamRender(out);
		//version 0
		out<<mBaseRadius<<mTopRadius<<mHeight<<mSlices<<mStacks<<mHasBottomBase<<mHasTopBase;
	}
}

/*---------------------------------------------------------------
	Implements the reading from a CStream capability of
		CSerializable objects
  ---------------------------------------------------------------*/
void CCylinder::readFromStream(CStream &in,int version)	{
	switch (version)	{
		case 0:
			readFromStreamRender(in);
			in>>mBaseRadius>>mTopRadius>>mHeight>>mSlices>>mStacks>>mHasBottomBase>>mHasTopBase;
			break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

bool solveEqn(double a,double b,double c,double &t)	{	//Actually, the b from the quadratic equation is the DOUBLE of this. But this way, operations are simpler.
	if (a<0)	{
		a=-a;
		b=-b;
		c=-c;
	}
	if (a>=mrpt::math::geometryEpsilon)	{
		double delta=square(b)-a*c;
		if (delta==0) return (t=-b/a)>=0;
		else if (delta>=0)	{
			delta=sqrt(delta);
			if (-b-delta>0)	{
				t=(-b-delta)/a;
				return true;
			}	else if (-b+delta>0)	{
				t=(-b+delta)/a;
				return true;
			}	//else return false;	Both solutions are negative
		}	//else return false;	Both solutions are complex
	}	else if (abs(b)>=geometryEpsilon)	 {
		t=-c/(b+b);
		return t>=0;
	}	//else return false;	This actually isn't an equation
	return false;
}

bool CCylinder::traceRay(const mrpt::poses::CPose3D &o,double &dist) const	{
	TLine3D lin;
	createFromPoseX(o-this->m_pose,lin);
	lin.unitarize();	//By adding this line, distance from any point of the line to its base is exactly equal to the "t".
	if (abs(lin.director[2])<geometryEpsilon)	{
		if (!reachesHeight(lin.pBase.z)) return false;
		float r;
		return getRadius(static_cast<float>(lin.pBase.z),r)?solveEqn(square(lin.director[0])+square(lin.director[1]),lin.director[0]*lin.pBase.x+lin.director[1]*lin.pBase.y,square(lin.pBase.x)+square(lin.pBase.y)-square(r),dist):false;
	}
	bool fnd=false;
	double nDist,tZ0;
	if (mHasBottomBase&&(tZ0=-lin.pBase.z/lin.director[2])>0)	{
		nDist=sqrt(square(lin.pBase.x+tZ0*lin.director[0])+square(lin.pBase.y+tZ0*lin.director[1]));
		if (nDist<=mBaseRadius)	{
			fnd=true;
			dist=tZ0;
		}
	}
	if (mHasTopBase)	{
		tZ0=(mHeight-lin.pBase.z)/lin.director[2];
		if (tZ0>0&&(!fnd||tZ0<dist))	{
			nDist=sqrt(square(lin.pBase.x+tZ0*lin.director[0])+square(lin.pBase.y+tZ0*lin.director[1]));
			if (nDist<=mTopRadius)	{
				fnd=true;
				dist=tZ0;
			}
		}
	}
	if (mBaseRadius==mTopRadius)	{
		if (solveEqn(square(lin.director[0])+square(lin.director[1]),lin.director[0]*lin.pBase.x+lin.director[1]*lin.pBase.y,square(lin.pBase.x)+square(lin.pBase.y)-square(mBaseRadius),nDist)) if ((!fnd||nDist<dist)&&reachesHeight(lin.pBase.z+nDist*lin.director[2]))	{
			dist=nDist;
			fnd=true;
		}
	}	else	{
		double slope=(mTopRadius-mBaseRadius)/mHeight;
		if (solveEqn(square(lin.director[0])+square(lin.director[1])-square(lin.director[2]*slope),lin.pBase.x*lin.director[0]+lin.pBase.y*lin.director[1]-(mBaseRadius+slope*lin.pBase.z)*slope*lin.director[2],square(lin.pBase.x)+square(lin.pBase.y)-square(mBaseRadius+slope*lin.pBase.z),nDist)) if ((!fnd||nDist<dist)&&reachesHeight(lin.pBase.z+nDist*lin.director[2]))	{
			dist=nDist;
			fnd=true;
		}
	}
	return fnd;
}
