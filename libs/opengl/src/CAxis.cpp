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


#include <mrpt/opengl/CAxis.h>

#include "opengl_internals.h"


using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::system;
using namespace mrpt::utils;
using namespace std;

IMPLEMENTS_SERIALIZABLE( CAxis, CRenderizableDisplayList, mrpt::opengl )


void   CAxis::render_dl() const
{
#if MRPT_HAS_OPENGL_GLUT
	MRPT_START

	glEnable (GL_BLEND);
	checkOpenGLError();
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	checkOpenGLError();

	ASSERT_(m_frecuency>=0);

    glLineWidth(m_lineWidth);
	checkOpenGLError();

    glBegin( GL_LINES );
    glColor4ub(m_color.R,m_color.G,m_color.B,m_color.A);
	//X axis
	glVertex3f( m_xmin, 0.0f, 0.0f );
    glVertex3f( m_xmax, 0.0f, 0.0f );
	//Y axis
	glVertex3f( 0.0f, m_ymin, 0.0f );
    glVertex3f( 0.0f, m_ymax, 0.0f);
	//Z axis
	glVertex3f( 0.0f, 0.0f, m_zmin );
    glVertex3f( 0.0f, 0.0f, m_zmax );

	glEnd();
	checkOpenGLError();

    glLineWidth(1.0f);
	checkOpenGLError();

	glDisable (GL_BLEND);
	checkOpenGLError();

	// Draw the "tick marks":
	if (m_marks ==true)
	{
		char n[50];

		// X axis
		glPushMatrix();
		glTranslatef(m_xmin,.0f,.05f);
		glRotatef(180,0,0,1);
		glRotatef(90,1,0,0);
		for (float i = m_xmin ; i<= m_xmax ; i = i + m_frecuency)
		{
			os::sprintf(n,50,"%.02f",i);
			gl_utils::glDrawText(n, 0.25 /* scale */,  mrpt::opengl::FILL );
			glTranslatef(-m_frecuency,0,0);
		}

		glPopMatrix();
		glPushMatrix();
		glTranslatef(m_xmax+0.5f*m_frecuency,0,0);
		glRotatef(180,0,0,1);
		glRotatef(90,1,0,0);
		gl_utils::glDrawText("+X", 0.3, mrpt::opengl::NICE );
		glPopMatrix();


		// Y axis
		glPushMatrix();
		glTranslatef(.0f,m_ymin,.05f);
		glRotatef(90,0,0,1);
		glRotatef(90,1,0,0);
		for (float i = m_ymin ; i<= m_ymax ; i = i + m_frecuency)
		{
			if (std::abs(i)>1e-4)
			{	// Dont draw the "0" more than once
				os::sprintf(n,50,"%.02f",i);
				gl_utils::glDrawText(n, 0.25 /* scale */,  mrpt::opengl::FILL );
			}
			glTranslatef(m_frecuency,0,0);
		}

		glPopMatrix();
		glPushMatrix();
		glTranslatef(0,m_ymax+1.0f*m_frecuency,0);
		glRotatef(-90,0,0,1);
		glRotatef(90,1,0,0);
		gl_utils::glDrawText("+Y", 0.3, mrpt::opengl::NICE );
		glPopMatrix();


		// Z axis
		glPushMatrix();
		glTranslatef(.0f,.0f,m_zmin);
		glRotatef(180,0,0,1);
		glRotatef(90,1,0,0);
		for (float i = m_zmin ; i<= m_zmax ; i = i + m_frecuency)
		{
			if (std::abs(i)>1e-4)
			{	// Dont draw the "0" more than once
				os::sprintf(n,50,"%.02f",i);
				gl_utils::glDrawText(n, 0.25 /* scale */,  mrpt::opengl::FILL );
			}
			glTranslatef(0,m_frecuency,0);
		}

		glPopMatrix();
		glPushMatrix();
		glTranslatef(0,0,m_zmax+0.5f*m_frecuency);
		glRotatef(180,0,0,1);
		glRotatef(90,1,0,0);
		gl_utils::glDrawText("+Z", 0.3, mrpt::opengl::NICE );
		glPopMatrix();

	}

	MRPT_END
/*******************************************************/
#endif
}

/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void  CAxis::writeToStream(CStream &out,int *version) const
{

	if (version)
		*version = 0;
	else
	{
		writeToStreamRender(out);
		out << m_xmin << m_ymin << m_zmin;
		out << m_xmax << m_ymax << m_zmax;
		out << m_frecuency << m_lineWidth << m_marks;
	}
}

/*---------------------------------------------------------------
	Implements the reading from a CStream capability of
		CSerializable objects
  ---------------------------------------------------------------*/
void  CAxis::readFromStream(CStream &in,int version)
{
	switch(version)
	{
	case 0:
		{
			readFromStreamRender(in);
			in >> m_xmin >> m_ymin >> m_zmin;
			in >> m_xmax >> m_ymax >> m_zmax;
			in >> m_frecuency >> m_lineWidth >> m_marks;
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
	CRenderizableDisplayList::notifyChange();
}

void CAxis::getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const
{
	bb_min.x = m_xmin;
	bb_min.y = m_ymin;
	bb_min.z = m_zmin;

	bb_max.x = m_xmax;
	bb_max.y = m_ymax;
	bb_max.z = m_zmax;

	// Convert to coordinates of my parent:
	m_pose.composePoint(bb_min, bb_min);
	m_pose.composePoint(bb_max, bb_max);
}
