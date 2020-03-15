/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/stl_serialization.h>

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CSetOfLines, CRenderizableShaderWireFrame, mrpt::opengl)

/** Constructor */
CSetOfLines::CSetOfLines()
{
	// Override default pointsize=1 in points shader:
	CRenderizableShaderPoints::m_pointSize = 0;
}

/** Constructor with a initial set of lines. */
CSetOfLines::CSetOfLines(const std::vector<TSegment3D>& sgms, bool antiAliasing)
	: m_Segments(sgms)
{
	// Override default pointsize=1 in points shader:
	CRenderizableShaderPoints::m_pointSize = 0;

	m_lineWidth = 1;
	m_antiAliasing = antiAliasing;
}

void CSetOfLines::render(const RenderContext& rc) const
{
	switch (rc.shader_id)
	{
		case DefaultShaderID::POINTS:
			CRenderizableShaderPoints::render(rc);
			break;
		case DefaultShaderID::WIREFRAME:
			CRenderizableShaderWireFrame::render(rc);
			break;
	};
}
void CSetOfLines::renderUpdateBuffers() const
{
	CRenderizableShaderPoints::renderUpdateBuffers();
	CRenderizableShaderWireFrame::renderUpdateBuffers();
}

/*---------------------------------------------------------------
							setLineByIndex
  ---------------------------------------------------------------*/
void CSetOfLines::setLineByIndex(
	size_t index, const mrpt::math::TSegment3D& segm)
{
	MRPT_START
	if (index >= m_Segments.size()) THROW_EXCEPTION("Index out of bounds");
	CRenderizable::notifyChange();
	m_Segments[index] = segm;
	MRPT_END
}

float CSetOfLines::getVerticesPointSize() const { return m_pointSize; }
void CSetOfLines::setVerticesPointSize(const float size_points)
{
	m_pointSize = size_points;
	CRenderizable::notifyChange();
}

void CSetOfLines::onUpdateBuffers_Wireframe()
{
	auto& vbd = CRenderizableShaderWireFrame::m_vertex_buffer_data;
	auto& cbd = CRenderizableShaderWireFrame::m_color_buffer_data;
	vbd.clear();
	vbd.reserve(m_Segments.size() * 2);

	for (const auto& segment : m_Segments)
	{
		vbd.emplace_back(segment.point1);
		vbd.emplace_back(segment.point2);
	}

	// The same color to all vertices:
	cbd.assign(vbd.size(), m_color);
}

void CSetOfLines::onUpdateBuffers_Points()
{
	auto& vbd = CRenderizableShaderPoints::m_vertex_buffer_data;
	auto& cbd = CRenderizableShaderPoints::m_color_buffer_data;
	vbd.clear();
	cbd.clear();

	// Only draw points if they are enabled:
	if (m_pointSize <= .0f) return;

	vbd.reserve(m_Segments.size() * 2);

	for (const auto& segment : m_Segments)
	{
		vbd.emplace_back(segment.point1);
		vbd.emplace_back(segment.point2);
	}

	// The same color to all vertices:
	cbd.assign(vbd.size(), m_color);
}

uint8_t CSetOfLines::serializeGetVersion() const { return 4; }
void CSetOfLines::serializeTo(mrpt::serialization::CArchive& out) const
{
	writeToStreamRender(out);
	out << m_Segments << m_lineWidth;
	out << m_antiAliasing;  // Added in v3
	CRenderizableShaderPoints::params_serialize(out);  // v4
}

void CSetOfLines::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		case 1:
		{
			readFromStreamRender(in);
			CVectorFloat x0, y0, z0, x1, y1, z1;
			in >> x0 >> y0 >> z0 >> x1 >> y1 >> z1;
			if (version >= 1)
				in >> m_lineWidth;
			else
				m_lineWidth = 1;
			size_t N = x0.size();
			m_Segments.resize(N);
			for (size_t i = 0; i < N; i++)
			{
				m_Segments[i][0][0] = x0[i];
				m_Segments[i][0][1] = y0[i];
				m_Segments[i][0][2] = z0[i];
				m_Segments[i][1][0] = x1[i];
				m_Segments[i][1][1] = y1[i];
				m_Segments[i][1][2] = z1[i];
			}
		}
		break;
		case 2:
		case 3:
		case 4:
		{
			readFromStreamRender(in);
			in >> m_Segments;
			in >> m_lineWidth;
			if (version >= 3)
				in >> m_antiAliasing;
			else
				m_antiAliasing = true;
			if (version >= 4)
				CRenderizableShaderPoints::params_deserialize(in);
			else
				m_pointSize = .0f;
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
	CRenderizable::notifyChange();
}

void CSetOfLines::getBoundingBox(
	mrpt::math::TPoint3D& bb_min, mrpt::math::TPoint3D& bb_max) const
{
	bb_min = mrpt::math::TPoint3D(
		std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
		std::numeric_limits<double>::max());
	bb_max = mrpt::math::TPoint3D(
		-std::numeric_limits<double>::max(),
		-std::numeric_limits<double>::max(),
		-std::numeric_limits<double>::max());

	for (const auto& s : m_Segments)
	{
		for (size_t p = 0; p < 2; p++)
		{
			const TPoint3D& pt = s[p];
			for (size_t j = 0; j < 3; j++)
			{
				keep_min(bb_min[j], pt[j]);
				keep_max(bb_max[j], pt[j]);
			}
		}
	}

	// Convert to coordinates of my parent:
	m_pose.composePoint(bb_min, bb_min);
	m_pose.composePoint(bb_max, bb_max);
}

void CSetOfLines::getLineByIndex(
	size_t index, double& x0, double& y0, double& z0, double& x1, double& y1,
	double& z1) const
{
	ASSERT_(index < m_Segments.size());
	const mrpt::math::TPoint3D& p0 = m_Segments[index].point1;
	const mrpt::math::TPoint3D& p1 = m_Segments[index].point2;
	x0 = p0.x;
	y0 = p0.y;
	z0 = p0.z;
	x1 = p1.x;
	y1 = p1.y;
	z1 = p1.z;
}
