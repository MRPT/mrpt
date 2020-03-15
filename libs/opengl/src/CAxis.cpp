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
#include <mrpt/opengl/CText3D.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/os.h>
#include "gltext.h"

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::system;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CAxis, CRenderizableShaderWireFrame, mrpt::opengl)

CAxis::CAxis(
	float xmin, float ymin, float zmin, float xmax, float ymax, float zmax,
	float frecuency, float lineWidth, bool marks)
	: m_xmin(xmin),
	  m_ymin(ymin),
	  m_zmin(zmin),
	  m_xmax(xmax),
	  m_ymax(ymax),
	  m_zmax(zmax),
	  m_frequency(frecuency)
{
	CRenderizableShaderWireFrame::setLineWidth(lineWidth);

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
	CRenderizable::notifyChange();
}

void CAxis::onUpdateBuffers_Wireframe()
{
	using mrpt::math::TPoint3Df;

	auto& vbd = CRenderizableShaderWireFrame::m_vertex_buffer_data;
	auto& cbd = CRenderizableShaderWireFrame::m_color_buffer_data;
	vbd.clear();

	m_gl_labels.clear();

	// X axis
	vbd.emplace_back(m_xmin, 0.0f, 0.0f);
	vbd.emplace_back(m_xmax, 0.0f, 0.0f);
	// Y axis
	vbd.emplace_back(0.0f, m_ymin, 0.0f);
	vbd.emplace_back(0.0f, m_ymax, 0.0f);
	// Z axis
	vbd.emplace_back(0.0f, 0.0f, m_zmin);
	vbd.emplace_back(0.0f, 0.0f, m_zmax);

	// Draw the "tick marks" for X,Y,Z
	const float ml = m_markLen * m_frequency;

	char n[50];
	const std::array<TPoint3Df, 3> init_trans = {
		{{m_xmin, .0f, ml}, {.0f, m_ymin, ml}, {.0f, .0f, m_zmin}}};
	const std::array<std::array<float, 2>, 3> xyz_ranges = {
		{{m_xmin, m_xmax}, {m_ymin, m_ymax}, {m_zmin, m_zmax}}};
	const std::array<TPoint3Df, 3> tick0 = {
		{{0, -ml, -ml}, {-ml, .0f, -ml}, {-ml, .0f, .0f}}};
	const std::array<TPoint3Df, 3> tick1 = {
		{{0, ml, -ml}, {ml, .0f, -ml}, {ml, .0f, .0f}}};
	const std::array<TPoint3Df, 3> endMark = {
		{{m_xmax + 1.0f * m_frequency, 0, 0},
		 {0, m_ymax + .5f * m_frequency, 0},
		 {0, 0, m_zmax + 0.5f * m_frequency}}};
	const std::array<const char*, 3> axis2name = {{"+X", "+Y", "+Z"}};

	for (unsigned int axis = 0; axis < 3; axis++)
	{
		if (!m_marks[axis]) continue;

		TPoint3Df tick_incr(0, 0, 0);
		tick_incr[axis] = m_frequency;

		const auto& tf = init_trans[axis];
		TPoint3Df cur_tf = tf;

		for (float i = xyz_ranges[axis][0]; i <= xyz_ranges[axis][1];
			 i = i + m_frequency, cur_tf = cur_tf + tick_incr)
		{
			// Dont draw the "0" more than once
			if (axis != 0 && std::abs(i) < 1e-4f) continue;

			os::sprintf(n, 50, "%.02f", i);
			mrpt::opengl::internal::glSetFont("mono");

			auto label = mrpt::opengl::CText3D::Create();
			label->setTextStyle(mrpt::opengl::FILL);
			label->setScale(m_textScale);

			label->setPose(mrpt::poses::CPose3D(
				cur_tf.x, cur_tf.y, cur_tf.z, mrpt::DEG2RAD(m_textRot[0][0]),
				mrpt::DEG2RAD(m_textRot[0][1]),
				mrpt::DEG2RAD(m_textRot[0][2])));
			label->setString(n);
			m_gl_labels.emplace_back(label);

			// tick line:
			vbd.emplace_back(cur_tf + tick0[axis]);
			vbd.emplace_back(cur_tf + tick1[axis]);
		}

		auto label = mrpt::opengl::CText3D::Create();
		label->setTextStyle(mrpt::opengl::FILL);
		label->setScale(m_textScale * 1.2f);
		label->setPose(mrpt::poses::CPose3D(
			endMark[axis].x, endMark[axis].y, endMark[axis].z,
			mrpt::DEG2RAD(m_textRot[0][0]), mrpt::DEG2RAD(m_textRot[0][1]),
			mrpt::DEG2RAD(m_textRot[0][2])));
		label->setString(axis2name[axis]);
		m_gl_labels.emplace_back(label);
	}

	cbd.assign(vbd.size(), m_color);

	for (auto& lb : m_gl_labels) lb->updateBuffers();
}

void CAxis::enqueForRenderRecursive(
	const mrpt::opengl::TRenderMatrices& state, RenderQueue& rq) const
{
	// Enque rendering all text labels:
	mrpt::opengl::enqueForRendering(m_gl_labels, state, rq);
}

void CAxis::render(const RenderContext& rc) const
{
	// Base lines render:
	CRenderizableShaderWireFrame::render(rc);

	// Do nothing for text labels: the enqueForRenderRecursive() does the actual
	// job.
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
	CRenderizable::notifyChange();
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
	CRenderizable::notifyChange();
}
float CAxis::getFrequency() const { return m_frequency; }
void CAxis::enableTickMarks(bool v)
{
	m_marks.fill(v);
	CRenderizable::notifyChange();
}
void CAxis::enableTickMarks(bool show_x, bool show_y, bool show_z)
{
	m_marks[0] = show_x;
	m_marks[1] = show_y;
	m_marks[2] = show_z;
	CRenderizable::notifyChange();
}
void CAxis::setTextScale(float f)
{
	ASSERT_(f > 0);
	m_textScale = f;
	CRenderizable::notifyChange();
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
	CRenderizable::notifyChange();
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
