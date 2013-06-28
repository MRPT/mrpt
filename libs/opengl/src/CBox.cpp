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
#include <mrpt/opengl/CBox.h>
#include <mrpt/math/geometry.h>

#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CBox,CRenderizableDisplayList,mrpt::opengl)

/*---------------------------------------------------------------
							render
  ---------------------------------------------------------------*/
void CBox::render_dl() const	{
#if MRPT_HAS_OPENGL_GLUT
	if (m_color.A!=255)
	{
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	}
    else
    {
		glEnable(GL_DEPTH_TEST);
		glDisable(GL_BLEND);
    }

	if (this->m_wireframe)
	{
		glDisable(GL_LIGHTING);
		// wireframe:
		glLineWidth(m_lineWidth); checkOpenGLError();
		glBegin(GL_LINE_STRIP);
		glColor4ub(m_color.R,m_color.G,m_color.B,m_color.A);

		glVertex3d(m_corner_min.x,m_corner_min.y,m_corner_min.z);
		glVertex3d(m_corner_max.x,m_corner_min.y,m_corner_min.z);
		glVertex3d(m_corner_max.x,m_corner_min.y,m_corner_max.z);
		glVertex3d(m_corner_min.x,m_corner_min.y,m_corner_max.z);
		glVertex3d(m_corner_min.x,m_corner_min.y,m_corner_min.z);

		glEnd();

		glBegin(GL_LINE_STRIP);
		glColor4ub(m_color.R,m_color.G,m_color.B,m_color.A);

		glVertex3d(m_corner_min.x,m_corner_max.y,m_corner_min.z);
		glVertex3d(m_corner_max.x,m_corner_max.y,m_corner_min.z);
		glVertex3d(m_corner_max.x,m_corner_max.y,m_corner_max.z);
		glVertex3d(m_corner_min.x,m_corner_max.y,m_corner_max.z);
		glVertex3d(m_corner_min.x,m_corner_max.y,m_corner_min.z);

		glEnd();

		glBegin(GL_LINE_STRIP);
		glColor4ub(m_color.R,m_color.G,m_color.B,m_color.A);
		glVertex3d(m_corner_min.x,m_corner_min.y,m_corner_min.z);
		glVertex3d(m_corner_min.x,m_corner_max.y,m_corner_min.z);
		glVertex3d(m_corner_min.x,m_corner_max.y,m_corner_max.z);
		glVertex3d(m_corner_min.x,m_corner_min.y,m_corner_max.z);
		glEnd();

		glBegin(GL_LINE_STRIP);
		glColor4ub(m_color.R,m_color.G,m_color.B,m_color.A);
		glVertex3d(m_corner_max.x,m_corner_min.y,m_corner_min.z);
		glVertex3d(m_corner_max.x,m_corner_max.y,m_corner_min.z);
		glVertex3d(m_corner_max.x,m_corner_max.y,m_corner_max.z);
		glVertex3d(m_corner_max.x,m_corner_min.y,m_corner_max.z);
		glEnd();

		glEnable(GL_LIGHTING);
	}
	else
	{
		// solid:
		glBegin(GL_TRIANGLES);
		glColor4ub(m_color.R,m_color.G,m_color.B,m_color.A);

		// Front face:
		gl_utils::renderTriangleWithNormal(
			TPoint3D(m_corner_max.x,m_corner_min.y,m_corner_min.z),
			TPoint3D(m_corner_min.x,m_corner_min.y,m_corner_min.z),
			TPoint3D(m_corner_max.x,m_corner_min.y,m_corner_max.z) );
		gl_utils::renderTriangleWithNormal(
			TPoint3D(m_corner_min.x,m_corner_min.y,m_corner_min.z),
			TPoint3D(m_corner_min.x,m_corner_min.y,m_corner_max.z),
			TPoint3D(m_corner_max.x,m_corner_min.y,m_corner_max.z) );

		// Back face:
		gl_utils::renderTriangleWithNormal(
			TPoint3D(m_corner_max.x,m_corner_max.y,m_corner_min.z),
			TPoint3D(m_corner_min.x,m_corner_max.y,m_corner_min.z),
			TPoint3D(m_corner_max.x,m_corner_max.y,m_corner_max.z) );
		gl_utils::renderTriangleWithNormal(
			TPoint3D(m_corner_min.x,m_corner_max.y,m_corner_min.z),
			TPoint3D(m_corner_min.x,m_corner_max.y,m_corner_max.z),
			TPoint3D(m_corner_max.x,m_corner_max.y,m_corner_max.z) );

		// Left face:
		gl_utils::renderTriangleWithNormal(
			TPoint3D(m_corner_min.x,m_corner_min.y,m_corner_min.z),
			TPoint3D(m_corner_min.x,m_corner_max.y,m_corner_min.z),
			TPoint3D(m_corner_min.x,m_corner_max.y,m_corner_max.z) );
		gl_utils::renderTriangleWithNormal(
			TPoint3D(m_corner_min.x,m_corner_min.y,m_corner_max.z),
			TPoint3D(m_corner_min.x,m_corner_min.y,m_corner_min.z),
			TPoint3D(m_corner_min.x,m_corner_max.y,m_corner_max.z) );

		// Right face:
		gl_utils::renderTriangleWithNormal(
			TPoint3D(m_corner_max.x,m_corner_min.y,m_corner_min.z),
			TPoint3D(m_corner_max.x,m_corner_max.y,m_corner_min.z),
			TPoint3D(m_corner_max.x,m_corner_max.y,m_corner_max.z) );
		gl_utils::renderTriangleWithNormal(
			TPoint3D(m_corner_max.x,m_corner_min.y,m_corner_max.z),
			TPoint3D(m_corner_max.x,m_corner_min.y,m_corner_min.z),
			TPoint3D(m_corner_max.x,m_corner_max.y,m_corner_max.z) );

		// Bottom face:
		gl_utils::renderTriangleWithNormal(
			TPoint3D(m_corner_min.x,m_corner_min.y,m_corner_min.z),
			TPoint3D(m_corner_max.x,m_corner_min.y,m_corner_min.z),
			TPoint3D(m_corner_max.x,m_corner_max.y,m_corner_min.z) );
		gl_utils::renderTriangleWithNormal(
			TPoint3D(m_corner_min.x,m_corner_max.y,m_corner_min.z),
			TPoint3D(m_corner_min.x,m_corner_min.y,m_corner_min.z),
			TPoint3D(m_corner_max.x,m_corner_max.y,m_corner_min.z) );

		// Top face:
		gl_utils::renderTriangleWithNormal(
			TPoint3D(m_corner_min.x,m_corner_min.y,m_corner_max.z),
			TPoint3D(m_corner_max.x,m_corner_min.y,m_corner_max.z),
			TPoint3D(m_corner_max.x,m_corner_max.y,m_corner_max.z) );
		gl_utils::renderTriangleWithNormal(
			TPoint3D(m_corner_min.x,m_corner_max.y,m_corner_max.z),
			TPoint3D(m_corner_min.x,m_corner_min.y,m_corner_max.z),
			TPoint3D(m_corner_max.x,m_corner_max.y,m_corner_max.z) );

		glEnd();
	}


	glDisable(GL_BLEND);

#endif
}

/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void CBox::writeToStream(CStream &out,int *version) const	{
	if (version) *version=0;
	else	{
		writeToStreamRender(out);
		//version 0
		out <<
			m_corner_min.x << m_corner_min.y << m_corner_min.z <<
			m_corner_max.x << m_corner_max.y << m_corner_max.z <<
			m_wireframe << m_lineWidth;
	}
}

/*---------------------------------------------------------------
	Implements the reading from a CStream capability of
		CSerializable objects
  ---------------------------------------------------------------*/
void CBox::readFromStream(CStream &in,int version)	{
	switch (version)	{
		case 0:
			readFromStreamRender(in);
			in >>
			m_corner_min.x >> m_corner_min.y >> m_corner_min.z >>
			m_corner_max.x >> m_corner_max.y >> m_corner_max.z >>
			m_wireframe >> m_lineWidth;
			break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
	CRenderizableDisplayList::notifyChange();
}

void CBox::setBoxCorners(const mrpt::math::TPoint3D &corner1, const mrpt::math::TPoint3D &corner2)
{
	CRenderizableDisplayList::notifyChange();

	// Order the coordinates so we always have the min/max in their right position:
	m_corner_min.x = std::min(corner1.x,corner2.x);
	m_corner_min.y = std::min(corner1.y,corner2.y);
	m_corner_min.z = std::min(corner1.z,corner2.z);

	m_corner_max.x = std::max(corner1.x,corner2.x);
	m_corner_max.y = std::max(corner1.y,corner2.y);
	m_corner_max.z = std::max(corner1.z,corner2.z);
}


bool CBox::traceRay(const mrpt::poses::CPose3D &o,double &dist) const
{
	THROW_EXCEPTION("TO DO")
}


void CBox::getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const
{
	bb_min = m_corner_min;
	bb_max = m_corner_max;

	// Convert to coordinates of my parent:
	m_pose.composePoint(bb_min, bb_min);
	m_pose.composePoint(bb_max, bb_max);
}
