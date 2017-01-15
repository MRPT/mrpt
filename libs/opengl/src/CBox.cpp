/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header
#include <mrpt/opengl/CBox.h>
#include <mrpt/math/geometry.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/opengl/gl_utils.h>

#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CBox,CRenderizableDisplayList,mrpt::opengl)

CBoxPtr CBox::Create(const mrpt::math::TPoint3D &corner1, const mrpt::math::TPoint3D &corner2, bool  is_wireframe , float lineWidth)	
{ 
	return CBoxPtr(new CBox(corner1,corner2,is_wireframe,lineWidth)); 
}
CBox::CBox() :
	m_corner_min(-1,-1,-1),
	m_corner_max(1,1,1),
	m_wireframe(false),
	m_lineWidth(1),
	m_draw_border(false),
	m_solidborder_color(0,0,0)
{
}

CBox::CBox(const mrpt::math::TPoint3D &corner1, const mrpt::math::TPoint3D &corner2, bool  is_wireframe, float lineWidth) :
	m_wireframe(is_wireframe) ,
	m_lineWidth( lineWidth ),
	m_draw_border(false),
	m_solidborder_color(0,0,0)
{
	setBoxCorners(corner1,corner2);
}

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

	if (!m_wireframe)
	{
		// solid:
		glEnable(GL_NORMALIZE);

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
		glDisable(GL_NORMALIZE);
	}

	if (m_wireframe || m_draw_border)
	{
		glDisable(GL_LIGHTING);

		if (m_draw_border)
		{
			glEnable(GL_BLEND);
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		}

		// wireframe:
		glLineWidth(m_lineWidth); checkOpenGLError();

		mrpt::math::TPoint3D  	a = m_corner_min, b = m_corner_max;

		if (m_wireframe)
			glColor4ub(m_color.R,m_color.G,m_color.B,m_color.A);
		else
		{
			glColor4ub(m_solidborder_color.R,m_solidborder_color.G,m_solidborder_color.B,m_solidborder_color.A);

			// Draw lines "a bit" far above the solid surface:
		/*	mrpt::math::TPoint3D d = b-a;
			d*=0.001;
			a-=d; b+=d;*/
		}

		glBegin(GL_LINE_STRIP);
		glVertex3d(a.x,a.y,a.z);
		glVertex3d(b.x,a.y,a.z);
		glVertex3d(b.x,a.y,b.z);
		glVertex3d(a.x,a.y,b.z);
		glVertex3d(a.x,a.y,a.z);
		glEnd();

		glBegin(GL_LINE_STRIP);
		glVertex3d(a.x,b.y,a.z);
		glVertex3d(b.x,b.y,a.z);
		glVertex3d(b.x,b.y,b.z);
		glVertex3d(a.x,b.y,b.z);
		glVertex3d(a.x,b.y,a.z);
		glEnd();

		glBegin(GL_LINE_STRIP);
		glVertex3d(a.x,a.y,a.z);
		glVertex3d(a.x,b.y,a.z);
		glVertex3d(a.x,b.y,b.z);
		glVertex3d(a.x,a.y,b.z);
		glEnd();

		glBegin(GL_LINE_STRIP);
		glVertex3d(b.x,a.y,a.z);
		glVertex3d(b.x,b.y,a.z);
		glVertex3d(b.x,b.y,b.z);
		glVertex3d(b.x,a.y,b.z);
		glEnd();

		glEnable(GL_LIGHTING);
	}

	glDisable(GL_BLEND);

#endif
}

/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void CBox::writeToStream(mrpt::utils::CStream &out,int *version) const	{
	if (version) *version=1;
	else	{
		writeToStreamRender(out);
		//version 0
		out <<
			m_corner_min.x << m_corner_min.y << m_corner_min.z <<
			m_corner_max.x << m_corner_max.y << m_corner_max.z <<
			m_wireframe << m_lineWidth;
		// Version 1:
		out << m_draw_border << m_solidborder_color;
	}
}

/*---------------------------------------------------------------
	Implements the reading from a CStream capability of
		CSerializable objects
  ---------------------------------------------------------------*/
void CBox::readFromStream(mrpt::utils::CStream &in,int version)	{
	switch (version)	{
		case 0:
		case 1:
			readFromStreamRender(in);
			in >>
			m_corner_min.x >> m_corner_min.y >> m_corner_min.z >>
			m_corner_max.x >> m_corner_max.y >> m_corner_max.z >>
			m_wireframe >> m_lineWidth;
			// Version 1:
			if (version>=1)
				in >> m_draw_border >> m_solidborder_color;
			else
			{
				m_draw_border = false;
			}

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
	MRPT_UNUSED_PARAM(o); MRPT_UNUSED_PARAM(dist);
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
