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

#include <mrpt/opengl/COpenGLStandardObject.h>
#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::poses;
using namespace mrpt::utils;

IMPLEMENTS_SERIALIZABLE(COpenGLStandardObject,CRenderizableDisplayList,mrpt::opengl)

#define COMPILE_TIME_ASSERT(N,expr)   \
	char dummy_constraint##N[expr]

#if MRPT_HAS_OPENGL_GLUT
COMPILE_TIME_ASSERT(GLENUM,sizeof(GLenum)==sizeof(_GLENUM));
#endif


/*---------------------------------------------------------------
							render
  ---------------------------------------------------------------*/
void renderFunc(TPoint3D p)	{
#if MRPT_HAS_OPENGL_GLUT
	glVertex3f(p.x,p.y,p.z);
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
		vector<TPoint3D>::const_iterator it=vertices.begin();
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
void COpenGLStandardObject::writeToStream(CStream &out,int *version) const	{
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
void COpenGLStandardObject::readFromStream(CStream &in,int version)	{
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

bool COpenGLStandardObject::traceRay(const mrpt::poses::CPose3D &o,float &dist) const	{
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

