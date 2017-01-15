/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/opengl/COpenGLStandardObject.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/utils/stl_serialization.h>
#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace mrpt::utils;

IMPLEMENTS_SERIALIZABLE(COpenGLStandardObject,CRenderizableDisplayList,mrpt::opengl)

#define COMPILE_TIME_ASSERT(N,expr)   \
	char dummy_constraint##N[expr]

#if MRPT_HAS_OPENGL_GLUT
COMPILE_TIME_ASSERT(GLENUM,sizeof(GLenum)==sizeof(_GLENUM));
#endif

COpenGLStandardObjectPtr COpenGLStandardObject::Create(_GLENUM t,const std::vector<TPoint3D> &v,uint32_t cs,const std::vector<_GLENUM> &en)	
{
	if (cs!=0&&v.size()%cs!=0) throw std::logic_error("Vertices vector does not match chunk size");
	return COpenGLStandardObjectPtr(new COpenGLStandardObject(t,v,cs,en));
}

/*---------------------------------------------------------------
							render
  ---------------------------------------------------------------*/
void renderFunc(TPoint3D p)	{
#if MRPT_HAS_OPENGL_GLUT
	glVertex3f(p.x,p.y,p.z);
#else
	MRPT_UNUSED_PARAM(p);
#endif
}

void COpenGLStandardObject::render_dl()	const	{
#if MRPT_HAS_OPENGL_GLUT
	for_each(enabled.begin(),enabled.end(),glEnable);
	glShadeModel(GL_SMOOTH);
	//This line won't take any effect if GL_BLEND is not enabled, so it's safe to always execute it.
	glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
	glColor4ub(m_color.R,m_color.G,m_color.B,m_color.A);
	if (normal[0]||normal[1]||normal[2]) glNormal3f(normal[0],normal[1],normal[2]);
	if (chunkSize==0)	{
		glBegin(type);
		for_each(vertices.begin(),vertices.end(),renderFunc);
		glEnd();
	}	else	{
		std::vector<TPoint3D>::const_iterator it=vertices.begin();
		do	{
			glBegin(type);
			for_each(it,it+chunkSize,renderFunc);
			it+=chunkSize;
			glEnd();
		}	while (it!=vertices.end());
	}
	for_each(enabled.begin(),enabled.end(),glDisable);
#endif
}
/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void COpenGLStandardObject::writeToStream(mrpt::utils::CStream &out,int *version) const	{
	if (version) *version=1;
	else	{
		writeToStreamRender(out);
		out<< type <<vertices<<chunkSize<<enabled;
	}
}

/*---------------------------------------------------------------
	Implements the reading from a CStream capability of
		CSerializable objects
  ---------------------------------------------------------------*/
void COpenGLStandardObject::readFromStream(mrpt::utils::CStream &in,int version)	{
	switch (version)	{
		case 1:	{
				readFromStreamRender(in);
				in>>type>>vertices>>chunkSize>>enabled;
			}
			break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
	CRenderizableDisplayList::notifyChange();
}

bool COpenGLStandardObject::traceRay(const mrpt::poses::CPose3D &o,double &dist) const	{
	MRPT_UNUSED_PARAM(o); MRPT_UNUSED_PARAM(dist);
	//This object isn't intended to hold geometric properties. No trace ray should be performed on it.
	return false;
}


void COpenGLStandardObject::getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const
{
	bb_min.x = 0;
	bb_min.y = 0;
	bb_min.z = 0;

	bb_max.x = 0;
	bb_max.y = 0;
	bb_max.z = 0;

	// Convert to coordinates of my parent:
	m_pose.composePoint(bb_min, bb_min);
	m_pose.composePoint(bb_max, bb_max);
}


void COpenGLStandardObject::disable(_GLENUM flag) 
{
	std::vector<_GLENUM>::iterator it = enabled.begin();
	while (it!=enabled.end())
	{
		if (*it==flag)
				it = enabled.erase(it);
		else ++it;
	}
	CRenderizableDisplayList::notifyChange();
}
