/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/math/TLine3D.h>
#include <mrpt/math/geometry.h>
#include <mrpt/opengl/CCylinder.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSchemeArchiveBase.h>

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CCylinder, CRenderizable, mrpt::opengl)

void CCylinder::onUpdateBuffers_Triangles()
{
	auto& tris = CRenderizableShaderTriangles::m_triangles;
	tris.clear();

	// precomputed table:
	ASSERT_ABOVE_(m_slices, 2);

	const float dAng = 2 * M_PIf / m_slices;
	float a = 0;
	// unit circle points: cos(ang),sin(ang)
	std::vector<mrpt::math::TPoint2Df> circle(m_slices);
	for (unsigned int i = 0; i < m_slices; i++, a += dAng)
	{
		circle[i].x = cos(a);
		circle[i].y = sin(a);
	}

	const float r0 = m_baseRadius, r1 = m_topRadius;

	// cylinder walls:
	for (unsigned int i = 0; i < m_slices; i++)
	{
		const auto ip = (i + 1) % m_slices;

		tris.emplace_back(
			TPoint3Df(r0 * circle[i].x, r0 * circle[i].y, .0f),
			TPoint3Df(r0 * circle[ip].x, r0 * circle[ip].y, .0f),
			TPoint3Df(r1 * circle[i].x, r1 * circle[i].y, m_height));

		tris.emplace_back(
			TPoint3Df(r0 * circle[ip].x, r0 * circle[ip].y, .0f),
			TPoint3Df(r1 * circle[ip].x, r1 * circle[ip].y, m_height),
			TPoint3Df(r1 * circle[i].x, r1 * circle[i].y, m_height));
	}

	// bottom & top disks:
	for (unsigned int i = 0; i < m_slices; i++)
	{
		const auto ip = (i + 1) % m_slices;
		tris.emplace_back(
			TPoint3Df(r0 * circle[i].x, r0 * circle[i].y, .0f),
			TPoint3Df(r0 * circle[ip].x, r0 * circle[ip].y, .0f),
			TPoint3Df(.0f, .0f, .0f));

		tris.emplace_back(
			TPoint3Df(r1 * circle[i].x, r1 * circle[i].y, m_height),
			TPoint3Df(r1 * circle[ip].x, r1 * circle[ip].y, m_height),
			TPoint3Df(.0f, .0f, m_height));
	}

	// All faces, same color:
	for (auto& t : tris) t.setColor(m_color);
}

void CCylinder::serializeTo(mrpt::serialization::CSchemeArchiveBase& out) const
{
	SCHEMA_SERIALIZE_DATATYPE_VERSION(1);
	out["baseRadius"] = m_baseRadius;
	out["topRadius"] = m_topRadius;
	out["height"] = m_height;
	out["slices"] = m_slices;
	out["hasBottomBase"] = m_hasBottomBase;
	out["hasTopBase"] = m_hasTopBase;
}
void CCylinder::serializeFrom(mrpt::serialization::CSchemeArchiveBase& in)
{
	uint8_t version;
	SCHEMA_DESERIALIZE_DATATYPE_VERSION();
	switch (version)
	{
		case 1:
		{
			m_baseRadius = static_cast<float>(in["baseRadius"]);
			m_topRadius = static_cast<float>(in["topRadius"]);
			m_height = static_cast<float>(in["height"]);
			m_slices = static_cast<uint32_t>(in["slices"]);
			m_hasBottomBase = static_cast<bool>(in["hasBottomBase"]);
			m_hasTopBase = static_cast<bool>(in["hasTopBase"]);
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	}
}
uint8_t CCylinder::serializeGetVersion() const { return 1; }
void CCylinder::serializeTo(mrpt::serialization::CArchive& out) const
{
	writeToStreamRender(out);
	// version 0
	out << m_baseRadius << m_topRadius << m_height << m_slices
		<< m_hasBottomBase << m_hasTopBase;
}
void CCylinder::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		case 1:
			readFromStreamRender(in);
			in >> m_baseRadius >> m_topRadius >> m_height >> m_slices;

			if (version < 1)
			{
				float old_mStacks;
				in >> old_mStacks;
			}

			in >> m_hasBottomBase >> m_hasTopBase;
			break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
	CRenderizable::notifyChange();
}

bool solveEqn(double a, double b, double c, double& t)
{  // Actually, the b from the quadratic equation is the DOUBLE of this. But
	// this way, operations are simpler.
	if (a < 0)
	{
		a = -a;
		b = -b;
		c = -c;
	}
	if (a >= mrpt::math::getEpsilon())
	{
		double delta = square(b) - a * c;
		if (delta == 0)
			return (t = -b / a) >= 0;
		else if (delta >= 0)
		{
			delta = sqrt(delta);
			if (-b - delta > 0)
			{
				t = (-b - delta) / a;
				return true;
			}
			else if (-b + delta > 0)
			{
				t = (-b + delta) / a;
				return true;
			}  // else return false;	Both solutions are negative
		}  // else return false;	Both solutions are complex
	}
	else if (std::abs(b) >= mrpt::math::getEpsilon())
	{
		t = -c / (b + b);
		return t >= 0;
	}  // else return false;	This actually isn't an equation
	return false;
}

bool CCylinder::traceRay(const mrpt::poses::CPose3D& o, double& dist) const
{
	TLine3D lin;
	mrpt::math::createFromPoseX((o - this->m_pose).asTPose(), lin);
	lin.unitarize();  // By adding this line, distance from any point of the

	const float zz = d2f(lin.pBase.z);

	// line to its base is exactly equal to the "t".
	if (std::abs(lin.director[2]) < getEpsilon())
	{
		if (!reachesHeight(zz)) return false;
		float r;
		return getRadius(zz, r)
				   ? solveEqn(
						 square(lin.director[0]) + square(lin.director[1]),
						 lin.director[0] * lin.pBase.x +
							 lin.director[1] * lin.pBase.y,
						 square(lin.pBase.x) + square(lin.pBase.y) - square(r),
						 dist)
				   : false;
	}
	bool fnd = false;
	double nDist, tZ0;
	if (m_hasBottomBase && (tZ0 = -lin.pBase.z / lin.director[2]) > 0)
	{
		nDist = sqrt(
			square(lin.pBase.x + tZ0 * lin.director[0]) +
			square(lin.pBase.y + tZ0 * lin.director[1]));
		if (nDist <= m_baseRadius)
		{
			fnd = true;
			dist = tZ0;
		}
	}
	if (m_hasTopBase)
	{
		tZ0 = (m_height - lin.pBase.z) / lin.director[2];
		if (tZ0 > 0 && (!fnd || tZ0 < dist))
		{
			nDist = sqrt(
				square(lin.pBase.x + tZ0 * lin.director[0]) +
				square(lin.pBase.y + tZ0 * lin.director[1]));
			if (nDist <= m_topRadius)
			{
				fnd = true;
				dist = tZ0;
			}
		}
	}
	if (m_baseRadius == m_topRadius)
	{
		if (solveEqn(
				square(lin.director[0]) + square(lin.director[1]),
				lin.director[0] * lin.pBase.x + lin.director[1] * lin.pBase.y,
				square(lin.pBase.x) + square(lin.pBase.y) -
					square(m_baseRadius),
				nDist))
			if ((!fnd || nDist < dist) &&
				reachesHeight(lin.pBase.z + nDist * lin.director[2]))
			{
				dist = nDist;
				fnd = true;
			}
	}
	else
	{
		double slope = (m_topRadius - m_baseRadius) / m_height;
		if (solveEqn(
				square(lin.director[0]) + square(lin.director[1]) -
					square(lin.director[2] * slope),
				lin.pBase.x * lin.director[0] + lin.pBase.y * lin.director[1] -
					(m_baseRadius + slope * lin.pBase.z) * slope *
						lin.director[2],
				square(lin.pBase.x) + square(lin.pBase.y) -
					square(m_baseRadius + slope * lin.pBase.z),
				nDist))
			if ((!fnd || nDist < dist) &&
				reachesHeight(lin.pBase.z + nDist * lin.director[2]))
			{
				dist = nDist;
				fnd = true;
			}
	}
	return fnd;
}

void CCylinder::getBoundingBox(
	mrpt::math::TPoint3D& bb_min, mrpt::math::TPoint3D& bb_max) const
{
	bb_min.x = -std::max(m_baseRadius, m_topRadius);
	bb_min.y = bb_min.x;
	bb_min.z = 0;

	bb_max.x = std::max(m_baseRadius, m_topRadius);
	bb_max.y = bb_max.x;
	bb_max.z = m_height;

	// Convert to coordinates of my parent:
	m_pose.composePoint(bb_min, bb_min);
	m_pose.composePoint(bb_max, bb_max);
}
