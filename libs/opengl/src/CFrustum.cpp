/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
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
#include <mrpt/opengl/CFrustum.h>

#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CFrustum,CRenderizableDisplayList,mrpt::opengl)

/*---------------------------------------------------------------
							render
  ---------------------------------------------------------------*/
void CFrustum::render_dl() const	{
#if MRPT_HAS_OPENGL_GLUT
	if (m_color.A!=255 || (m_draw_planes && m_planes_color.A!=255) )
	{
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	}
    else
    {
		glEnable(GL_DEPTH_TEST);
		glDisable(GL_BLEND);
    }

    // Compute the 8 corners of the frustum:
    TPoint3Df pts[8];
	for (int j=0;j<2;j++)
	{
    	const float r = j==0 ? m_min_distance : m_max_distance;
    	for (int i=0;i<4;i++)
			pts[4*j+i].x = r;
		pts[4*j+0].y = -r*sin(m_fov_horz_left);
		pts[4*j+1].y = -r*sin(m_fov_horz_left);
		pts[4*j+2].y =  r*sin(m_fov_horz_right);
		pts[4*j+3].y =  r*sin(m_fov_horz_right);
		pts[4*j+0].z = -r*sin(m_fov_vert_down);
		pts[4*j+1].z =  r*sin(m_fov_vert_up);
		pts[4*j+2].z = -r*sin(m_fov_vert_down);
		pts[4*j+3].z =  r*sin(m_fov_vert_up);
    }

    // Render lines:
	if (m_draw_lines)
	{
		glDisable(GL_LIGHTING);

		const int draw_path[] = {
			0,1,3,2,0,4,6,2,
			3,7,6,4,5,7,5,1 };

		// wireframe:
		glLineWidth(m_lineWidth); checkOpenGLError();
		glBegin(GL_LINE_STRIP);
		glColor4ub(m_color.R,m_color.G,m_color.B,m_color.A);

		for (size_t i=0;i<sizeof(draw_path)/sizeof(draw_path[0]);i++)
			glVertex3fv(&pts[draw_path[i]].x);

		glEnd();

	}

	if (m_draw_planes)
	{
		// solid:
		glBegin(GL_TRIANGLES);
		glColor4ub(m_planes_color.R,m_planes_color.G,m_planes_color.B,m_planes_color.A);

		gl_utils::renderQuadWithNormal( pts[0], pts[2], pts[6], pts[4] );
		gl_utils::renderQuadWithNormal( pts[2], pts[3], pts[7], pts[6] );
		gl_utils::renderQuadWithNormal( pts[4], pts[6], pts[7], pts[5] );
		gl_utils::renderQuadWithNormal( pts[1], pts[5], pts[7], pts[3] );
		gl_utils::renderQuadWithNormal( pts[1], pts[5], pts[7], pts[3] );
		gl_utils::renderQuadWithNormal( pts[4], pts[5], pts[1], pts[0] );

		glEnd();
	}

	glDisable(GL_BLEND);

#endif
}

//Ctors
CFrustum::CFrustum():
	m_min_distance(0.1f),
	m_max_distance(1.f),
	m_fov_horz_left(mrpt::utils::DEG2RAD(45)),
	m_fov_horz_right(mrpt::utils::DEG2RAD(45)),
	m_fov_vert_down(mrpt::utils::DEG2RAD(30)),
	m_fov_vert_up(mrpt::utils::DEG2RAD(30)),
	m_draw_lines(true),
	m_draw_planes(false),
	m_lineWidth(1.5f),
	m_planes_color(0xE0,0x00,0x00, 0x50)  // RGBA
{
	keep_min( m_fov_horz_left, DEG2RAD(89.9f) ); keep_max(m_fov_horz_left, 0);
	keep_min( m_fov_horz_right, DEG2RAD(89.9f) ); keep_max(m_fov_horz_right, 0);
	keep_min( m_fov_vert_down, DEG2RAD(89.9f) ); keep_max(m_fov_vert_down, 0);
	keep_min( m_fov_vert_up, DEG2RAD(89.9f) ); keep_max(m_fov_vert_up, 0);
}

CFrustum::CFrustum(float near_distance, float far_distance, float horz_FOV_degrees, float vert_FOV_degrees, float lineWidth, bool draw_lines, bool draw_planes) :
	m_min_distance(near_distance),
	m_max_distance(far_distance),
	m_fov_horz_left(mrpt::utils::DEG2RAD(.5f*horz_FOV_degrees)),
	m_fov_horz_right(mrpt::utils::DEG2RAD(.5f*horz_FOV_degrees)),
	m_fov_vert_down(mrpt::utils::DEG2RAD(.5f*vert_FOV_degrees)),
	m_fov_vert_up(mrpt::utils::DEG2RAD(.5f*vert_FOV_degrees)),
	m_draw_lines(draw_lines),
	m_draw_planes(draw_planes),
	m_lineWidth(lineWidth),
	m_planes_color(0xE0,0x00,0x00, 0x50)  // RGBA
{
}


/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void CFrustum::writeToStream(CStream &out,int *version) const	{
	if (version) *version=0;
	else	{
		writeToStreamRender(out);
		//version 0
		out << m_min_distance << m_max_distance
		    << m_fov_horz_left << m_fov_horz_right
		    << m_fov_vert_down << m_fov_vert_up
			<< m_draw_lines << m_draw_planes
		    << m_lineWidth
		    << m_planes_color.R << m_planes_color.G << m_planes_color.B << m_planes_color.A;
	}
}

/*---------------------------------------------------------------
	Implements the reading from a CStream capability of
		CSerializable objects
  ---------------------------------------------------------------*/
void CFrustum::readFromStream(CStream &in,int version)	{
	switch (version)	{
		case 0:
			readFromStreamRender(in);
			in  >> m_min_distance >> m_max_distance
				>> m_fov_horz_left >> m_fov_horz_right
				>> m_fov_vert_down >> m_fov_vert_up
				>> m_draw_lines >> m_draw_planes
				>> m_lineWidth
				>> m_planes_color.R >> m_planes_color.G >> m_planes_color.B >> m_planes_color.A;
			break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}


bool CFrustum::traceRay(const mrpt::poses::CPose3D &o,double &dist) const
{
	THROW_EXCEPTION("TO DO")
}


// setters:
void CFrustum::setNearFarPlanes(const float near_distance, const float far_distance)
{
	m_min_distance = near_distance;
	m_max_distance = far_distance;
	CRenderizableDisplayList::notifyChange();
}
void CFrustum::setHorzFOV(const float fov_horz_degrees)
{
	m_fov_horz_right = m_fov_horz_left = 0.5f* mrpt::utils::DEG2RAD(fov_horz_degrees);
	keep_min( m_fov_horz_left, DEG2RAD(89.9f) ); keep_max(m_fov_horz_left, 0);
	keep_min( m_fov_horz_right, DEG2RAD(89.9f) ); keep_max(m_fov_horz_right, 0);
	CRenderizableDisplayList::notifyChange();
}
void CFrustum::setVertFOV(const float fov_vert_degrees)
{
	m_fov_vert_down=m_fov_vert_up = 0.5f*mrpt::utils::DEG2RAD(fov_vert_degrees);
	keep_min( m_fov_vert_down, DEG2RAD(89.9f) ); keep_max(m_fov_vert_down, 0);
	keep_min( m_fov_vert_up, DEG2RAD(89.9f) ); keep_max(m_fov_vert_up, 0);
	CRenderizableDisplayList::notifyChange();
}
void CFrustum::setHorzFOVAsymmetric(const float fov_horz_left_degrees,const float fov_horz_right_degrees)
{
	m_fov_horz_left= mrpt::utils::DEG2RAD(fov_horz_left_degrees);
	m_fov_horz_right = mrpt::utils::DEG2RAD(fov_horz_right_degrees);
	keep_min( m_fov_horz_left, DEG2RAD(89.9f) ); keep_max(m_fov_horz_left, 0);
	keep_min( m_fov_horz_right, DEG2RAD(89.9f) ); keep_max(m_fov_horz_right, 0);
	CRenderizableDisplayList::notifyChange();
}
void CFrustum::setVertFOVAsymmetric(const float fov_vert_down_degrees,const float fov_vert_up_degrees)
{
	m_fov_vert_down= mrpt::utils::DEG2RAD(fov_vert_down_degrees);
	m_fov_vert_up  = mrpt::utils::DEG2RAD(fov_vert_up_degrees);
	keep_min( m_fov_vert_down, DEG2RAD(89.9f) ); keep_max(m_fov_vert_down, 0);
	keep_min( m_fov_vert_up, DEG2RAD(89.9f) ); keep_max(m_fov_vert_up, 0);
	CRenderizableDisplayList::notifyChange();
}
