/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/opengl/CSetOfTexturedTriangles.h>
#include <mrpt/serialization/CArchive.h>

#include <mrpt/opengl/opengl_api.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::math;

IMPLEMENTS_SERIALIZABLE(CSetOfTexturedTriangles, CRenderizable, mrpt::opengl)

void CSetOfTexturedTriangles::onUpdateBuffers_TexturedTriangles()
{
	// Nothing else to do: all data is already in m_triangles in my base class.
}

uint8_t CSetOfTexturedTriangles::serializeGetVersion() const { return 2; }
void CSetOfTexturedTriangles::serializeTo(
	mrpt::serialization::CArchive& out) const
{
	uint32_t n;

	writeToStreamRender(out);
	writeToStreamTexturedObject(out);

	n = (uint32_t)m_triangles.size();

	out << n;

	for (uint32_t i = 0; i < n; i++) m_triangles[i].writeTo(out);
}

void CSetOfTexturedTriangles::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		case 1:
		case 2:
		{
			readFromStreamRender(in);
			if (version >= 2)
			{
				readFromStreamTexturedObject(in);
			}
			else
			{  // Old version.
				THROW_EXCEPTION("deserializing old version not supported.");
			}

			uint32_t n;
			in >> n;
			m_triangles.resize(n);

			for (uint32_t i = 0; i < n; i++) m_triangles[i].readFrom(in);
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
	CRenderizable::notifyChange();
}

bool CSetOfTexturedTriangles::traceRay(
	const mrpt::poses::CPose3D& o, double& dist) const
{
	MRPT_UNUSED_PARAM(o);
	MRPT_UNUSED_PARAM(dist);
	throw std::runtime_error(
		"TODO: TraceRay not implemented in CSetOfTexturedTriangles");
}

void CSetOfTexturedTriangles::getBoundingBox(
	mrpt::math::TPoint3D& bb_min, mrpt::math::TPoint3D& bb_max) const
{
	bb_min = mrpt::math::TPoint3D(
		std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
		std::numeric_limits<double>::max());
	bb_max = mrpt::math::TPoint3D(
		-std::numeric_limits<double>::max(),
		-std::numeric_limits<double>::max(),
		-std::numeric_limits<double>::max());

	for (const auto& t : m_triangles)
	{
		for (size_t i = 0; i < 3; i++)
		{
			keep_min(bb_min.x, t.x(i));
			keep_max(bb_max.x, t.x(i));

			keep_min(bb_min.y, t.y(i));
			keep_max(bb_max.y, t.y(i));

			keep_min(bb_min.z, t.z(i));
			keep_max(bb_max.z, t.z(i));
		}
	}

	// Convert to coordinates of my parent:
	m_pose.composePoint(bb_min, bb_min);
	m_pose.composePoint(bb_max, bb_max);
}
