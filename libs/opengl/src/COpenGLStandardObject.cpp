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
}

bool COpenGLStandardObject::traceRay(const mrpt::poses::CPose3D &o,float &dist) const	{
	//This object isn't intended to hold geometric properties. No trace ray should be performed on it.
	return false;
}
