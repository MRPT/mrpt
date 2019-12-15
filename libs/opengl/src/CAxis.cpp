/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/opengl/CAxis.h>
#include <mrpt/opengl/gl_utils.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/os.h>

#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::system;

using namespace std;

IMPLEMENTS_SERIALIZABLE(CAxis, CRenderizableDisplayList, mrpt::opengl)

CAxis::CAxis(
	float xmin, float ymin, float zmin, float xmax, float ymax, float zmax,
	float frecuency, float lineWidth, bool marks)
	: m_xmin(xmin),
	  m_ymin(ymin),
	  m_zmin(zmin),
	  m_xmax(xmax),
	  m_ymax(ymax),
	  m_zmax(zmax),
	  m_frequency(frecuency),
	  m_lineWidth(lineWidth)

{
	m_marks.fill(marks);

	// x:180, 0, 90
	m_textRot[0][0] = 180.f;
	m_textRot[0][1] = 0.f;
	m_textRot[0][2] = 90.f;
	// y:90, 0, 90
	m_textRot[1][0] = 90.f;
	m_textRot[1][1] = 0.f;
	m_textRot[1][2] = 90.f;
	// z:180, 0, 90
	m_textRot[2][0] = 180.f;
	m_textRot[2][1] = 0.f;
	m_textRot[2][2] = 90.f;
}

void CAxis::setTickMarksLength(float len)
{
	m_markLen = len;
	CRenderizableDisplayList::notifyChange();
}

void CAxis::render_dl() const
{
#if MRPT_HAS_OPENGL_GLUT
	MRPT_START
	glDisable(GL_LIGHTING);

	glEnable(GL_BLEND);
	CHECK_OPENGL_ERROR();
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	CHECK_OPENGL_ERROR();

	ASSERT_(m_frequency >= 0);

	glLineWidth(m_lineWidth);
	CHECK_OPENGL_ERROR();
	glBegin(GL_LINES);
	glColor4ub(m_color.R, m_color.G, m_color.B, m_color.A);
	// X axis
	glVertex3f(m_xmin, 0.0f, 0.0f);
	glVertex3f(m_xmax, 0.0f, 0.0f);
	// Y axis
	glVertex3f(0.0f, m_ymin, 0.0f);
	glVertex3f(0.0f, m_ymax, 0.0f);
	// Z axis
	glVertex3f(0.0f, 0.0f, m_zmin);
	glVertex3f(0.0f, 0.0f, m_zmax);

	glEnd();
	CHECK_OPENGL_ERROR();

	glLineWidth(1.0f);
	CHECK_OPENGL_ERROR();

	glDisable(GL_BLEND);
	CHECK_OPENGL_ERROR();

	// Draw the "tick marks" for X,Y,Z
	const float ml = m_markLen * m_frequency;

	char n[50];
	const std::array<mrpt::math::TPoint3Df, 3> init_trans = {
		{{m_xmin, .0f, ml}, {.0f, m_ymin, ml}, {.0f, .0f, m_zmin}}};
	const std::array<std::array<float, 2>, 3> xyz_ranges = {
		{{m_xmin, m_xmax}, {m_ymin, m_ymax}, {m_zmin, m_zmax}}};
	const std::array<mrpt::math::TPoint3Df, 3> tick0 = {
		{{0, -ml, -ml}, {-ml, .0f, -ml}, {-ml, .0f, .0f}}};
	const std::array<mrpt::math::TPoint3Df, 3> tick1 = {
		{{0, ml, -ml}, {ml, .0f, -ml}, {ml, .0f, .0f}}};
	const std::array<mrpt::math::TPoint3Df, 3> endMark = {
		{{m_xmax + 1.0f * m_frequency, 0, 0},
		 {0, m_ymax + .5f * m_frequency, 0},
		 {0, 0, m_zmax + 0.5f * m_frequency}}};
	const std::array<const char*, 3> axis2name = {{"+X", "+Y", "+Z"}};

	for (int axis = 0; axis < 3; axis++)
	{
		if (!m_marks[axis]) continue;

		glPushMatrix();
		const auto& tf = init_trans[axis];
		glTranslatef(tf.x, tf.y, tf.z);
		for (float i = xyz_ranges[axis][0]; i <= xyz_ranges[axis][1];
			 i = i + m_frequency)
		{
			// Dont draw the "0" more than once
			if (axis == 0 || std::abs(i) > 1e-4)
			{
				os::sprintf(n, 50, "%.02f", i);
				glPushMatrix();
				glRotatef(m_textRot[0][0], 0, 0, 1);
				glRotatef(m_textRot[0][1], 0, 1, 0);
				glRotatef(m_textRot[0][2], 1, 0, 0);
				gl_utils::glDrawText(n, m_textScale, mrpt::opengl::FILL);
				glBegin(GL_LINES);
				glVertex3f(tick0[axis].x, tick0[axis].y, tick0[axis].z);
				glVertex3f(tick1[axis].x, tick1[axis].y, tick1[axis].z);
				glEnd();
				glPopMatrix();
				CHECK_OPENGL_ERROR();
			}
			glTranslatef(
				axis == 0 ? m_frequency : 0, axis == 1 ? m_frequency : 0,
				axis == 2 ? m_frequency : 0);
		}

		glPopMatrix();
		glPushMatrix();
		glTranslatef(endMark[axis].x, endMark[axis].y, endMark[axis].z);
		glRotatef(m_textRot[0][0], 0, 0, 1);
		glRotatef(m_textRot[0][1], 0, 1, 0);
		glRotatef(m_textRot[0][2], 1, 0, 0);
		gl_utils::glDrawText(
			axis2name[axis], m_textScale * 1.2, mrpt::opengl::NICE);
		glPopMatrix();
	}

	glEnable(GL_LIGHTING);
	MRPT_END
/*******************************************************/
#endif
}

uint8_t CAxis::serializeGetVersion() const { return 2; }
void CAxis::serializeTo(mrpt::serialization::CArchive& out) const
{
	writeToStreamRender(out);
	out << m_xmin << m_ymin << m_zmin;
	out << m_xmax << m_ymax << m_zmax;
	out << m_frequency << m_lineWidth;
	// v1:
	out << m_marks[0] << m_marks[1] << m_marks[2] << m_textScale;
	for (auto i : m_textRot)
		for (int j = 0; j < 3; j++) out << i[j];
	// v2:
	out << m_markLen;
}

void CAxis::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		case 1:
		case 2:
		{
			readFromStreamRender(in);
			in >> m_xmin >> m_ymin >> m_zmin;
			in >> m_xmax >> m_ymax >> m_zmax;
			in >> m_frequency >> m_lineWidth;
			if (version >= 1)
			{
				in >> m_marks[0] >> m_marks[1] >> m_marks[2] >> m_textScale;
				for (auto& i : m_textRot)
					for (int j = 0; j < 3; j++) in >> i[j];
			}
			else
			{
				bool v;
				in >> v;
				m_marks.fill(v);
				m_textScale = 0.25f;
			}
			if (version >= 2) in >> m_markLen;
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
	CRenderizableDisplayList::notifyChange();
}

void CAxis::getBoundingBox(
	mrpt::math::TPoint3D& bb_min, mrpt::math::TPoint3D& bb_max) const
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
	ASSERT_(f > 0);
	m_frequency = f;
	CRenderizableDisplayList::notifyChange();
}
float CAxis::getFrequency() const { return m_frequency; }
void CAxis::setLineWidth(float w)
{
	m_lineWidth = w;
	CRenderizableDisplayList::notifyChange();
}
float CAxis::getLineWidth() const { return m_lineWidth; }
void CAxis::enableTickMarks(bool v)
{
	m_marks.fill(v);
	CRenderizableDisplayList::notifyChange();
}
void CAxis::enableTickMarks(bool show_x, bool show_y, bool show_z)
{
	m_marks[0] = show_x;
	m_marks[1] = show_y;
	m_marks[2] = show_z;
	CRenderizableDisplayList::notifyChange();
}
void CAxis::setTextScale(float f)
{
	ASSERT_(f > 0);
	m_textScale = f;
	CRenderizableDisplayList::notifyChange();
}
float CAxis::getTextScale() const { return m_textScale; }
void CAxis::setAxisLimits(
	float xmin, float ymin, float zmin, float xmax, float ymax, float zmax)
{
	m_xmin = xmin;
	m_ymin = ymin;
	m_zmin = zmin;
	m_xmax = xmax;
	m_ymax = ymax;
	m_zmax = zmax;
	CRenderizableDisplayList::notifyChange();
}
void CAxis::setTextLabelOrientation(
	int axis, float yaw_deg, float pitch_deg, float roll_deg)
{
	ASSERT_(axis >= 0 && axis < 3);
	m_textRot[axis][0] = yaw_deg;
	m_textRot[axis][1] = pitch_deg;
	m_textRot[axis][2] = roll_deg;
}
void CAxis::getTextLabelOrientation(
	int axis, float& yaw_deg, float& pitch_deg, float& roll_deg) const
{
	ASSERT_(axis >= 0 && axis < 3);
	yaw_deg = m_textRot[axis][0];
	pitch_deg = m_textRot[axis][1];
	roll_deg = m_textRot[axis][2];
}
