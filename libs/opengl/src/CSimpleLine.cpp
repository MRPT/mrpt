/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"	 // Precompiled header
//
#include <mrpt/opengl/CSimpleLine.h>
#include <mrpt/opengl/opengl_api.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CSimpleLine, CRenderizableShaderWireFrame, mrpt::opengl)

CSimpleLine::CSimpleLine(
	float x0, float y0, float z0, float x1, float y1, float z1, float lineWidth,
	bool antiAliasing)
	: m_x0(x0), m_y0(y0), m_z0(z0), m_x1(x1), m_y1(y1), m_z1(z1)
{
	m_lineWidth = lineWidth;
	m_antiAliasing = antiAliasing;
}

void CSimpleLine::onUpdateBuffers_Wireframe()
{
	auto& vbd = CRenderizableShaderWireFrame::m_vertex_buffer_data;
	std::unique_lock<std::shared_mutex> wfWriteLock(
		CRenderizableShaderWireFrame::m_wireframeMtx.data);

	vbd.resize(2);

	vbd[0] = {m_x0, m_y0, m_z0};
	vbd[1] = {m_x1, m_y1, m_z1};

	// The same color to all vertices:
	m_color_buffer_data.assign(vbd.size(), getColor_u8());
}

uint8_t CSimpleLine::serializeGetVersion() const { return 1; }
void CSimpleLine::serializeTo(mrpt::serialization::CArchive& out) const
{
	writeToStreamRender(out);
	out << m_x0 << m_y0 << m_z0;
	out << m_x1 << m_y1 << m_z1 << m_lineWidth;
	out << m_antiAliasing;	// Added in v1
}

void CSimpleLine::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 1:
		{
			readFromStreamRender(in);
			in >> m_x0 >> m_y0 >> m_z0;
			in >> m_x1 >> m_y1 >> m_z1 >> m_lineWidth;
			if (version >= 1) in >> m_antiAliasing;
			else
				m_antiAliasing = true;
		}
		break;
		default: MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
	CRenderizable::notifyChange();
}

auto CSimpleLine::internalBoundingBoxLocal() const -> mrpt::math::TBoundingBoxf
{
	return mrpt::math::TBoundingBoxf::FromUnsortedPoints(
		{std::min(m_x0, m_x1), std::min(m_y0, m_y1), std::min(m_z0, m_z1)},
		{std::max(m_x0, m_x1), std::max(m_y0, m_y1), std::max(m_z0, m_z1)});
}
