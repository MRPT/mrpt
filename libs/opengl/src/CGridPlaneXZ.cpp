/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/opengl/CGridPlaneXZ.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt;
using namespace mrpt::opengl;
using namespace std;

IMPLEMENTS_SERIALIZABLE(
	CGridPlaneXZ, CRenderizableShaderWireFrame, mrpt::opengl)

/** Constructor */
CGridPlaneXZ::CGridPlaneXZ(
	float xMin, float xMax, float zMin, float zMax, float y, float frequency,
	float lineWidth, bool antiAliasing)
	: m_xMin(xMin),
	  m_xMax(xMax),
	  m_zMin(zMin),
	  m_zMax(zMax),
	  m_plane_y(y),
	  m_frequency(frequency)
{
	m_lineWidth = lineWidth;
	m_antiAliasing = antiAliasing;
}

void CGridPlaneXZ::onUpdateBuffers()
{
	// Generate vertices:
	m_vertex_buffer_data.clear();
	m_color_buffer_data.clear();

	for (float z = m_zMin; z <= m_zMax; z += m_frequency)
	{
		m_vertex_buffer_data.emplace_back(m_xMin, m_plane_y, z);
		m_vertex_buffer_data.emplace_back(m_xMax, m_plane_y, z);
	}

	for (float x = m_xMin; x <= m_xMax; x += m_frequency)
	{
		m_vertex_buffer_data.emplace_back(x, m_plane_y, m_zMin);
		m_vertex_buffer_data.emplace_back(x, m_plane_y, m_zMax);
	}
	// The same color to all vertices:
	m_color_buffer_data.assign(m_vertex_buffer_data.size(), m_color);
}

uint8_t CGridPlaneXZ::serializeGetVersion() const { return 1; }
void CGridPlaneXZ::serializeTo(mrpt::serialization::CArchive& out) const
{
	writeToStreamRender(out);
	out << m_xMin << m_xMax;
	out << m_zMin << m_zMax << m_plane_y;
	out << m_frequency;
	out << m_lineWidth << m_antiAliasing;  // v1
}

void CGridPlaneXZ::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		case 1:
		{
			readFromStreamRender(in);
			in >> m_xMin >> m_xMax;
			in >> m_zMin >> m_zMax >> m_plane_y;
			in >> m_frequency;
			if (version >= 1)
				in >> m_lineWidth >> m_antiAliasing;
			else
			{
				m_lineWidth = 1.0f;
				m_antiAliasing = true;
			}
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
	CRenderizable::notifyChange();
}

void CGridPlaneXZ::getBoundingBox(
	mrpt::math::TPoint3D& bb_min, mrpt::math::TPoint3D& bb_max) const
{
	bb_min.x = m_xMin;
	bb_min.y = 0;
	bb_min.z = m_zMin;

	bb_max.x = m_xMax;
	bb_max.y = 0;
	bb_max.z = m_zMax;

	// Convert to coordinates of my parent:
	m_pose.composePoint(bb_min, bb_min);
	m_pose.composePoint(bb_max, bb_max);
}
