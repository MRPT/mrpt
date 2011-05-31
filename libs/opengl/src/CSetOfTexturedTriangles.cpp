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

#include <mrpt/opengl/CSetOfTexturedTriangles.h>
#include <mrpt/math/utils.h>

#include "opengl_internals.h"

using namespace std;
using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::utils;
using namespace mrpt::math;

IMPLEMENTS_SERIALIZABLE( CSetOfTexturedTriangles, CRenderizableDisplayList, mrpt::opengl )

/*---------------------------------------------------------------
							~CTexturedPlane
  ---------------------------------------------------------------*/
CSetOfTexturedTriangles::~CSetOfTexturedTriangles()
{
}

/*---------------------------------------------------------------
							render
  ---------------------------------------------------------------*/
void   CSetOfTexturedTriangles::render_texturedobj() const
{
#if MRPT_HAS_OPENGL_GLUT
	MRPT_START

	glShadeModel(GL_SMOOTH);

	glBegin(GL_TRIANGLES);

	float ax, ay, az, bx, by, bz;

	vector<TTriangle>::const_iterator	it;
	for (it = m_triangles.begin(); it != m_triangles.end(); it++)
	{
		// Compute the normal vector:
		// ---------------------------------
		ax = it->m_v2.m_x - it->m_v1.m_x;
		ay = it->m_v2.m_y - it->m_v1.m_y;
		az = it->m_v2.m_z - it->m_v1.m_z;

		bx = it->m_v3.m_x - it->m_v1.m_x;
		by = it->m_v3.m_y - it->m_v1.m_y;
		bz = it->m_v3.m_z - it->m_v1.m_z;

		glNormal3f(ay * bz - az * by, -ax * bz + az * bx, ax * by - ay * bx);

		glTexCoord2d(float(it->m_v1.m_u)/r_width, float(it->m_v1.m_v)/r_height); glVertex3f(it->m_v1.m_x, it->m_v1.m_y, it->m_v1.m_z);
		glTexCoord2d(float(it->m_v2.m_u)/r_width, float(it->m_v2.m_v)/r_height); glVertex3f(it->m_v2.m_x, it->m_v2.m_y, it->m_v2.m_z);
		glTexCoord2d(float(it->m_v3.m_u)/r_width, float(it->m_v3.m_v)/r_height); glVertex3f(it->m_v3.m_x, it->m_v3.m_y, it->m_v3.m_z);
	}

	glEnd();

	MRPT_END
#endif
}

/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void  CSetOfTexturedTriangles::writeToStream(CStream &out, int *version) const
{
	if (version)
		*version = 2;
	else
	{
		uint32_t n;

		writeToStreamRender(out);
		writeToStreamTexturedObject(out);

		n = (uint32_t)m_triangles.size();

		out << n;

		for (uint32_t i=0;i<n;i++)
			m_triangles[i].writeToStream(out);
	}
}

/*---------------------------------------------------------------
	Implements the reading from a CStream capability of
		CSerializable objects
  ---------------------------------------------------------------*/
void  CSetOfTexturedTriangles::readFromStream(CStream &in, int version)
{
	switch(version)
	{
	case 0:
	case 1:
	case 2:
		{
			readFromStreamRender(in);
			if (version>=2)
			{
				readFromStreamTexturedObject(in);
			}
			else
			{	// Old version.
				in >> CTexturedObject::m_textureImage;
				in >> CTexturedObject::m_enableTransparency;
				if (CTexturedObject::m_enableTransparency)
				{
					in >> CTexturedObject::m_textureImageAlpha;
					assignImage( CTexturedObject::m_textureImage, CTexturedObject::m_textureImageAlpha );
				}
				else
					assignImage( CTexturedObject::m_textureImage );
			}

			uint32_t n;
			in >> n;
			m_triangles.resize(n);

			for (uint32_t i=0;i<n;i++)
				m_triangles[i].readFromStream(in);

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

bool CSetOfTexturedTriangles::traceRay(const mrpt::poses::CPose3D &o, double &dist) const
{
	throw std::runtime_error("TODO: TraceRay not implemented in CSetOfTexturedTriangles");
}
