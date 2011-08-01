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

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
    glShadeModel(GL_SMOOTH);

	if (this->m_wireframe)
	{
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
