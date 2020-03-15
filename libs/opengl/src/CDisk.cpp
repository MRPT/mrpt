/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/opengl/CDisk.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt;
using namespace mrpt::opengl;
using namespace std;
using mrpt::poses::CPose3D;

IMPLEMENTS_SERIALIZABLE(CDisk, CRenderizable, mrpt::opengl)

void CDisk::onUpdateBuffers_Triangles()
{
	using mrpt::math::TPoint3Df;

	auto& tris = CRenderizableShaderTriangles::m_triangles;
	tris.clear();

	// precomputed table:
	ASSERT_ABOVE_(m_nSlices, 2);

	const float dAng = 2 * M_PIf / m_nSlices;
	float a = 0;
	// unit circle points: cos(ang),sin(ang)
	std::vector<mrpt::math::TPoint2Df> circle(m_nSlices);
	for (unsigned int i = 0; i < m_nSlices; i++, a += dAng)
	{
		circle[i].x = cos(a);
		circle[i].y = sin(a);
	}

	const float r0 = m_radiusIn, r1 = m_radiusOut;

	if (std::abs(r0) < 1e-6f)
	{
		// a filled disk:
		for (unsigned int i = 0; i < m_nSlices; i++)
		{
			const auto ip = (i + 1) % m_nSlices;
			tris.emplace_back(
				TPoint3Df(r1 * circle[i].x, r1 * circle[i].y, .0f),
				TPoint3Df(r1 * circle[ip].x, r1 * circle[ip].y, .0f),
				TPoint3Df(.0f, .0f, .0f));
		}
	}
	else
	{
		// A ring:
		for (unsigned int i = 0; i < m_nSlices; i++)
		{
			const auto ip = (i + 1) % m_nSlices;
			tris.emplace_back(
				TPoint3Df(r1 * circle[i].x, r1 * circle[i].y, .0f),
				TPoint3Df(r1 * circle[ip].x, r1 * circle[ip].y, .0f),
				TPoint3Df(r0 * circle[i].x, r0 * circle[i].y, .0f));

			tris.emplace_back(
				TPoint3Df(r1 * circle[ip].x, r1 * circle[ip].y, .0f),
				TPoint3Df(r0 * circle[ip].x, r0 * circle[ip].y, .0f),
				TPoint3Df(r0 * circle[i].x, r0 * circle[i].y, .0f));
		}
	}

	// All faces, same color:
	for (auto& t : tris) t.setColor(m_color);
}

uint8_t CDisk::serializeGetVersion() const { return 1; }
void CDisk::serializeTo(mrpt::serialization::CArchive& out) const
{
	writeToStreamRender(out);
	out << m_radiusIn << m_radiusOut;
	out << m_nSlices;
}

void CDisk::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		case 1:
		{
			readFromStreamRender(in);
			in >> m_radiusIn >> m_radiusOut;
			in >> m_nSlices;
			if (version < 1)
			{
				float dummy_loops;
				in >> dummy_loops;
			}
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
	CRenderizable::notifyChange();
}

bool CDisk::traceRay(const mrpt::poses::CPose3D& o, double& dist) const
{
	// The disk is contained initially in a plane which contains (0,0,0),
	// (1,0,0) and (0,1,0)
	// These points are converted into:
	//(x,y,z)
	//( cos(w)*cos(p)+x, sin(w)*cos(p)*y, -sin(p)+z )
	//( -sin(w)*cos(r)+cos(w)*sin(p)*sin(r)+x,
	// cos(w)*cos(r)+sin(w)*sin(p)*sin(r)+y, cos(p)*sin(r)*z )
	CPose3D transf = this->m_pose - o;
	double x = transf.x(), y = transf.y(), z = transf.z(), w = transf.yaw(),
		   p = transf.pitch(), r = transf.roll();
	double coef = sin(w) * sin(r) + cos(w) * sin(p) * cos(r);
	// coef is the first component of the normal to the transformed Z plane. So,
	// the scalar product between
	// this normal and (1,0,0) (which happens to be the beam's vector) equals
	// coef. And if it's 0, then both
	// are orthogonal, that is, the beam is parallel to the plane.
	if (coef == 0) return false;
	// The following expression yields the collision point between the plane and
	// the beam (the y and z
	// coordinates are zero).
	dist = x + (y * (sin(p) * sin(w) * cos(r) - cos(w) * sin(r)) +
				z * cos(p) * cos(r)) /
				   coef;
	if (dist < 0) return false;
	// Euclidean distance is invariant to rotations...
	double d2 = (x - dist) * (x - dist) + y * y + z * z;
	return d2 >= (m_radiusIn * m_radiusIn) && d2 <= (m_radiusOut * m_radiusOut);

	// IMPORTANT NOTICE: using geometric intersection between Z plane and
	// CPose's line intersection is SLOWER than the used method.
}

void CDisk::getBoundingBox(
	mrpt::math::TPoint3D& bb_min, mrpt::math::TPoint3D& bb_max) const
{
	bb_min.x = -std::max(m_radiusIn, m_radiusOut);
	bb_min.y = bb_min.x;
	bb_min.z = 0;

	bb_max.x = std::max(m_radiusIn, m_radiusOut);
	bb_max.y = bb_max.x;
	bb_max.z = 0;

	// Convert to coordinates of my parent:
	m_pose.composePoint(bb_min, bb_min);
	m_pose.composePoint(bb_max, bb_max);
}
