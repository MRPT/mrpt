/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/opengl/CAxis.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/system/os.h>
#include <mrpt/opengl/gl_utils.h>

#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::system;
using namespace mrpt::utils;
using namespace std;

IMPLEMENTS_SERIALIZABLE( CAxis, CRenderizableDisplayList, mrpt::opengl )

CAxis::CAxis(
	float xmin,float ymin, float zmin,
	float xmax, float ymax,  float zmax,
	float frecuency , float lineWidth , bool marks) :
	m_xmin(xmin),m_ymin(ymin),m_zmin(zmin),
	m_xmax(xmax),m_ymax(ymax),m_zmax(zmax),
	m_frequency(frecuency),
	m_lineWidth(lineWidth),
	m_textScale(0.25f)
{
	for (int i=0;i<3;i++) m_marks[i] = marks;
	
	//x:180, 0, 90
	m_textRot[0][0] = 180.f; m_textRot[0][1] = 0.f;  m_textRot[0][2] = 90.f; 
	//y:90, 0, 90
	m_textRot[1][0] =  90.f; m_textRot[1][1] = 0.f;  m_textRot[1][2] = 90.f; 
	//z:180, 0, 90
	m_textRot[2][0] = 180.f; m_textRot[2][1] = 0.f;  m_textRot[2][2] = 90.f; 
}


CAxisPtr CAxis::Create(
	float xmin,float ymin, float zmin,
	float xmax, float ymax,  float zmax,
	float frecuency, float lineWidth, bool marks)
{
	return CAxisPtr( new CAxis( xmin,ymin, zmin, xmax,ymax,zmax,frecuency,lineWidth,marks  ) );
}

void   CAxis::render_dl() const
{
#if MRPT_HAS_OPENGL_GLUT
	MRPT_START
	glDisable(GL_LIGHTING);

	glEnable (GL_BLEND);
	checkOpenGLError();
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	checkOpenGLError();

	ASSERT_(m_frequency>=0);

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
	char n[50];
	if (m_marks[0])
	{
		// X axis
		glPushMatrix();
		glTranslatef(m_xmin,.0f,.05f);
		for (float i = m_xmin ; i<= m_xmax ; i = i + m_frequency)
		{
			os::sprintf(n,50,"%.02f",i);
			glPushMatrix();
			glRotatef(m_textRot[0][0],0,0,1);
			glRotatef(m_textRot[0][1],0,1,0);
			glRotatef(m_textRot[0][2],1,0,0);
			gl_utils::glDrawText(n, m_textScale,  mrpt::opengl::FILL );
			glPopMatrix();
			glTranslatef(m_frequency,0,0);
		}

		glPopMatrix();
		glPushMatrix();
		glTranslatef(m_xmax+1.0f*m_frequency,0,0);
		glRotatef(m_textRot[0][0],0,0,1);
		glRotatef(m_textRot[0][1],0,1,0);
		glRotatef(m_textRot[0][2],1,0,0);
		gl_utils::glDrawText("+X", m_textScale*1.2, mrpt::opengl::NICE );
		glPopMatrix();
	}
	if (m_marks[1])
	{
		// Y axis
		glPushMatrix();
		glTranslatef(.0f,m_ymin,.05f);
		for (float i = m_ymin ; i<= m_ymax ; i = i + m_frequency)
		{
			if (std::abs(i)>1e-4)
			{	// Dont draw the "0" more than once
				os::sprintf(n,50,"%.02f",i);
				glPushMatrix();
				glRotatef(m_textRot[1][0],0,0,1);
				glRotatef(m_textRot[1][1],0,1,0);
				glRotatef(m_textRot[1][2],1,0,0);
				gl_utils::glDrawText(n, m_textScale,  mrpt::opengl::FILL );
				glPopMatrix();
			}
			glTranslatef(0,m_frequency,0);
		}

		glPopMatrix();
		glPushMatrix();
		glTranslatef(0,m_ymax+.5f*m_frequency,0);
		glRotatef(m_textRot[1][0],0,0,1);
		glRotatef(m_textRot[1][1],0,1,0);
		glRotatef(m_textRot[1][2],1,0,0);
		gl_utils::glDrawText("+Y", m_textScale*1.2, mrpt::opengl::NICE );
		glPopMatrix();
	}
	if (m_marks[2])
	{
		// Z axis
		glPushMatrix();
		glTranslatef(.0f,.0f,m_zmin);
		for (float i = m_zmin ; i<= m_zmax ; i = i + m_frequency)
		{
			if (std::abs(i)>1e-4)
			{	// Dont draw the "0" more than once
				glPushMatrix();
				glRotatef(m_textRot[2][0],0,0,1);
				glRotatef(m_textRot[2][1],0,1,0);
				glRotatef(m_textRot[2][2],1,0,0);
				os::sprintf(n,50,"%.02f",i);
				gl_utils::glDrawText(n, m_textScale,  mrpt::opengl::FILL );
				glPopMatrix();
			}
			glTranslatef(0,0,m_frequency);
		}

		glPopMatrix();
		glPushMatrix();
		glTranslatef(0,0,m_zmax+0.5f*m_frequency);
		glRotatef(m_textRot[2][0],0,0,1);
		glRotatef(m_textRot[2][1],0,1,0);
		glRotatef(m_textRot[2][2],1,0,0);
		gl_utils::glDrawText("+Z", m_textScale*1.2, mrpt::opengl::NICE );
		glPopMatrix();
	}

	glEnable(GL_LIGHTING);
	MRPT_END
/*******************************************************/
#endif
}

/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void  CAxis::writeToStream(mrpt::utils::CStream &out,int *version) const
{
	if (version)
		*version = 1;
	else
	{
		writeToStreamRender(out);
		out << m_xmin << m_ymin << m_zmin;
		out << m_xmax << m_ymax << m_zmax;
		out << m_frequency << m_lineWidth;
		// v1:
		out << m_marks[0] << m_marks[1] << m_marks[2] << m_textScale;
		for (int i=0;i<3;i++) for (int j=0;j<3;j++) out << m_textRot[i][j];
	}
}

/*---------------------------------------------------------------
	Implements the reading from a CStream capability of
		CSerializable objects
  ---------------------------------------------------------------*/
void  CAxis::readFromStream(mrpt::utils::CStream &in,int version)
{
	switch(version)
	{
	case 0:
	case 1:
		{
			readFromStreamRender(in);
			in >> m_xmin >> m_ymin >> m_zmin;
			in >> m_xmax >> m_ymax >> m_zmax;
			in >> m_frequency >> m_lineWidth;
			if (version>=1)
			{
				in >> m_marks[0] >> m_marks[1] >> m_marks[2] >> m_textScale;
				for (int i=0;i<3;i++) for (int j=0;j<3;j++) in >> m_textRot[i][j];
			}
			else {
				bool v;
				in >> v;
				for (int i=0;i<3;i++) m_marks[i] = v;
				m_textScale = 0.25f;
			}
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


void CAxis::setFrequency(float f) 
{ 
	ASSERT_(f>0);
	m_frequency=f;
	CRenderizableDisplayList::notifyChange(); 
}
float CAxis::getFrequency() const {
	return m_frequency;
}
void CAxis::setLineWidth(float w) { 
	m_lineWidth=w;
	CRenderizableDisplayList::notifyChange();
}
float CAxis::getLineWidth() const {
	return  m_lineWidth;
}

void CAxis::enableTickMarks(bool v) {
	for (int i=0;i<3;i++) m_marks[i]=v; 
	CRenderizableDisplayList::notifyChange(); 
}
void CAxis::enableTickMarks(bool show_x, bool show_y, bool show_z)
{
	m_marks[0]=show_x;
	m_marks[1]=show_y;
	m_marks[2]=show_z;
	CRenderizableDisplayList::notifyChange();
}
void CAxis::setTextScale(float f) {
	ASSERT_(f>0);
	m_textScale=f;
	CRenderizableDisplayList::notifyChange();
}
float CAxis::getTextScale() const {
	return m_textScale;
}

void CAxis::setAxisLimits(float xmin,float ymin, float zmin, float xmax,float ymax, float zmax)
{
	m_xmin=xmin; m_ymin=ymin; m_zmin=zmin;
	m_xmax=xmax; m_ymax=ymax; m_zmax=zmax;
	CRenderizableDisplayList::notifyChange();
}
void CAxis::setTextLabelOrientation(int axis, float yaw_deg, float pitch_deg, float roll_deg)
{
	ASSERT_(axis>=0 && axis<3);
	m_textRot[axis][0]=yaw_deg;
	m_textRot[axis][1]=pitch_deg;
	m_textRot[axis][2]=roll_deg;
}
void CAxis::getTextLabelOrientation(int axis, float &yaw_deg, float &pitch_deg, float &roll_deg) const
{
	ASSERT_(axis>=0 && axis<3);
	yaw_deg = m_textRot[axis][0];
	pitch_deg = m_textRot[axis][1];
	roll_deg = m_textRot[axis][2];
}
