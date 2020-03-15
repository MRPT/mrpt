/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/opengl/CSphere.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CSphere, CRenderizable, mrpt::opengl)

uint8_t CSphere::serializeGetVersion() const { return 2; }
void CSphere::serializeTo(mrpt::serialization::CArchive& out) const
{
	writeToStreamRender(out);
	out << m_radius;
	out << (uint32_t)m_nDivsLongitude << (uint32_t)m_nDivsLatitude;
}
void CSphere::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		case 1:
		case 2:
		{
			readFromStreamRender(in);
			in >> m_radius;
			uint32_t i, j;
			in >> i >> j;
			m_nDivsLongitude = i;
			m_nDivsLatitude = j;
			if (version == 1)
			{
				bool keepRadiusIndependentEyeDistance;
				in >> keepRadiusIndependentEyeDistance;
			}

			regenerateBaseParams();
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
	CRenderizable::notifyChange();
}

bool CSphere::traceRay(const mrpt::poses::CPose3D& o, double& dist) const
{
	// We need to find the points of the sphere which collide with the laser
	// beam.
	// The sphere's equation is invariant to rotations (but not to
	// translations), and we can take advantage of this;
	// we'll simply transform the center and then compute the beam's points
	// whose distance to that transformed point
	// equals the sphere's radius.

	CPose3D transf = this->m_pose - o;
	double x = transf.x(), y = transf.y(), z = transf.z();
	double r2 = m_radius * m_radius;
	double dyz = y * y + z * z;
	if (dyz > r2) return false;
	double dx = sqrt(r2 - dyz);
	if (x - dx >= 0)
	{
		dist = x - dx;
		return true;
	}
	else if (x + dx >= 0)
	{
		dist = x + dx;
		return true;
	}
	else
		return false;
}

void CSphere::getBoundingBox(
	mrpt::math::TPoint3D& bb_min, mrpt::math::TPoint3D& bb_max) const
{
	bb_min.x = -m_radius;
	bb_min.y = -m_radius;
	bb_min.z = -m_radius;

	bb_max.x = m_radius;
	bb_max.y = m_radius;
	bb_max.z = m_radius;

	// Convert to coordinates of my parent:
	m_pose.composePoint(bb_min, bb_min);
	m_pose.composePoint(bb_max, bb_max);
}

void CSphere::renderUpdateBuffers() const
{
	const_cast<CSphere*>(this)->regenerateBaseParams();
	BASE::renderUpdateBuffers();
}
