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

void CSetOfTexturedTriangles::getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const
{
	bb_min = mrpt::math::TPoint3D(std::numeric_limits<double>::max(),std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
	bb_max = mrpt::math::TPoint3D(-std::numeric_limits<double>::max(),-std::numeric_limits<double>::max(),-std::numeric_limits<double>::max());

	for (size_t i=0;i<m_triangles.size();i++) 
	{
		const TTriangle &t=m_triangles[i];

		keep_min(bb_min.x, t.m_v1.m_x);  keep_max(bb_max.x, t.m_v1.m_x);
		keep_min(bb_min.y, t.m_v1.m_y);  keep_max(bb_max.y, t.m_v1.m_y);
		keep_min(bb_min.z, t.m_v1.m_z);  keep_max(bb_max.z, t.m_v1.m_z);

		keep_min(bb_min.x, t.m_v2.m_x);  keep_max(bb_max.x, t.m_v2.m_x);
		keep_min(bb_min.y, t.m_v2.m_y);  keep_max(bb_max.y, t.m_v2.m_y);
		keep_min(bb_min.z, t.m_v2.m_z);  keep_max(bb_max.z, t.m_v2.m_z);

		keep_min(bb_min.x, t.m_v3.m_x);  keep_max(bb_max.x, t.m_v3.m_x);
		keep_min(bb_min.y, t.m_v3.m_y);  keep_max(bb_max.y, t.m_v3.m_y);
		keep_min(bb_min.z, t.m_v3.m_z);  keep_max(bb_max.z, t.m_v3.m_z);
	}

	// Convert to coordinates of my parent:
	m_pose.composePoint(bb_min, bb_min);
	m_pose.composePoint(bb_max, bb_max);
}
