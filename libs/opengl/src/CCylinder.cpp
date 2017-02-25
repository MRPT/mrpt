/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header
#include <mrpt/opengl/CCylinder.h>
#include <mrpt/math/geometry.h>
#include <mrpt/utils/CStream.h>

#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CCylinder,CRenderizableDisplayList,mrpt::opengl)

CCylinderPtr CCylinder::Create(const float baseRadius,const float topRadius,const float height,const int slices,const int stacks)	
{
	return CCylinderPtr(new CCylinder(baseRadius,topRadius,height,slices,stacks));
}
/*---------------------------------------------------------------
							render
  ---------------------------------------------------------------*/
void CCylinder::render_dl() const	{
#if MRPT_HAS_OPENGL_GLUT
	glEnable (GL_BLEND);
	checkOpenGLError();
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
	checkOpenGLError();
	GLUquadricObj *obj=gluNewQuadric();

	// This is required to draw cylinders of negative height.
	const float absHeight = std::abs(mHeight);
	if (mHeight<0)
	{
		glPushMatrix();
		glTranslatef(0,0,mHeight);
	}

	gluCylinder(obj,mBaseRadius,mTopRadius,absHeight,mSlices,mStacks);

	if (mHeight<0)
		glPopMatrix();

	if (mHasBottomBase) gluDisk(obj,0,mBaseRadius,mSlices,1);
	if (mHasTopBase&&mTopRadius>0)	{
		glPushMatrix();
		glTranslatef(0,0,mHeight);
		gluDisk(obj,0,mTopRadius,mSlices,1);
		glPopMatrix();
	}
	gluDeleteQuadric(obj);
	glDisable(GL_BLEND);

#endif
}

/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void CCylinder::writeToStream(mrpt::utils::CStream &out,int *version) const	{
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
void CCylinder::readFromStream(mrpt::utils::CStream &in,int version)	{
	switch (version)	{
		case 0:
			readFromStreamRender(in);
			in>>mBaseRadius>>mTopRadius>>mHeight>>mSlices>>mStacks>>mHasBottomBase>>mHasTopBase;
			break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
	CRenderizableDisplayList::notifyChange();
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
	}	else if (abs(b)>=mrpt::math::geometryEpsilon)	 {
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



void CCylinder::getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const
{
	bb_min.x = -std::max(mBaseRadius,mTopRadius);
	bb_min.y = bb_min.x;
	bb_min.z = 0;

	bb_max.x = std::max(mBaseRadius,mTopRadius);
	bb_max.y = bb_max.x;
	bb_max.z = mHeight;

	// Convert to coordinates of my parent:
	m_pose.composePoint(bb_min, bb_min);
	m_pose.composePoint(bb_max, bb_max);
}
