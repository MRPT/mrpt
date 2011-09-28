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


#include <mrpt/opengl/CTexturedPlane.h>
#include <mrpt/math/utils.h>
#include "opengl_internals.h"
#include <mrpt/poses/CPose3D.h>
#include <mrpt/opengl/CSetOfTriangles.h>

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::poses;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE( CTexturedPlane, CTexturedObject, mrpt::opengl )

/*---------------------------------------------------------------
							CTexturedPlane
  ---------------------------------------------------------------*/
CTexturedPlane::CTexturedPlane(
	float				x_min,
	float				x_max,
	float				y_min,
	float				y_max
	) :
		polygonUpToDate(false)
{
	// Copy data:
	m_xMin = x_min;
	m_xMax = x_max;
	m_yMin = y_min;
	m_yMax = y_max;
}


/*---------------------------------------------------------------
							~CTexturedPlane
  ---------------------------------------------------------------*/
CTexturedPlane::~CTexturedPlane()
{
}

/*---------------------------------------------------------------
							render
  ---------------------------------------------------------------*/
void   CTexturedPlane::render_texturedobj() const
{
#if MRPT_HAS_OPENGL_GLUT
	MRPT_START

	// Compute the exact texture coordinates:
	m_tex_x_min = 0;
	m_tex_x_max = 1.0f-((float)m_pad_x_right) / r_width;
	m_tex_y_min = 0;
	m_tex_y_max = 1.0f-((float)m_pad_y_bottom) / r_height;

	glBegin(GL_QUADS);

	glTexCoord2d(m_tex_x_min,m_tex_y_min);
	glVertex3f( m_xMin, m_yMin,0 );

	glTexCoord2d(m_tex_x_max,m_tex_y_min);
	glVertex3f( m_xMax, m_yMin,0 );

	glTexCoord2d(m_tex_x_max,m_tex_y_max);
	glVertex3f( m_xMax, m_yMax,0 );

	glTexCoord2d(m_tex_x_min,m_tex_y_max);
	glVertex3f( m_xMin, m_yMax,0 );

	glEnd();
	checkOpenGLError();

	MRPT_END
#endif
}

/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void  CTexturedPlane::writeToStream(CStream &out,int *version) const
{
	if (version)
		*version = 2;
	else
	{
		writeToStreamRender(out);

		out << m_xMin << m_xMax;
		out << m_yMin << m_yMax;

		writeToStreamTexturedObject(out);
	}
}

/*---------------------------------------------------------------
	Implements the reading from a CStream capability of
		CSerializable objects
  ---------------------------------------------------------------*/
void  CTexturedPlane::readFromStream(CStream &in,int version)
{
	switch(version)
	{
	case 0:
		{
			readFromStreamRender(in);
			in >> m_textureImage >> m_textureImageAlpha;
			in >> m_xMin >> m_xMax;
			in >> m_yMin >> m_yMax;

			assignImage( m_textureImage, m_textureImageAlpha );

		} break;
	case 1:
	case 2:
		{
			readFromStreamRender(in);

			in >> m_xMin >> m_xMax;
			in >> m_yMin >> m_yMax;

			if (version>=2)
			{
				readFromStreamTexturedObject(in);
			}
			else
			{	// Old version.
				in >> CTexturedObject::m_enableTransparency;
				in >> CTexturedObject::m_textureImage;
				if (CTexturedObject::m_enableTransparency)
				{
					in >> CTexturedObject::m_textureImageAlpha;
					assignImage( CTexturedObject::m_textureImage, CTexturedObject::m_textureImageAlpha );
				}
				else
					assignImage( CTexturedObject::m_textureImage );
			}

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
}

bool CTexturedPlane::traceRay(const mrpt::poses::CPose3D &o,double &dist) const	{
	if (!polygonUpToDate) updatePoly();
	return math::traceRay(tmpPoly,o-this->m_pose,dist);
}

void CTexturedPlane::updatePoly() const	{
	TPolygon3D poly(4);
	poly[0].x=poly[1].x=m_xMin;
	poly[2].x=poly[3].x=m_xMax;
	poly[0].y=poly[3].y=m_yMin;
	poly[1].y=poly[2].y=m_yMax;
	for (size_t i=0;i<4;i++) poly[i].z=0;
	tmpPoly.resize(1);
	tmpPoly[0]=poly;
	polygonUpToDate=true;
}
