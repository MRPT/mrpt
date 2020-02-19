/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/math/geometry.h>
#include <mrpt/opengl/CArrow.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSchemeArchiveBase.h>

#include <memory>

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CArrow, CRenderizableShaderTriangles, mrpt::opengl)

void CArrow::onUpdateBuffers_Triangles()
{
	using P3f = mrpt::math::TPoint3Df;

	auto& tris = CRenderizableShaderTriangles::m_triangles;
	tris.clear();

	// Compute the XYZ local frame of reference for the arrow:
	// XY radial, Z from Point #0 -> point #1:
	const P3f p0(m_x0, m_y0, m_z0), p1(m_x1, m_y1, m_z1);
	auto p = p1 - p0;
	const float P10_norm = p.norm();
	ASSERT_ABOVE_(P10_norm, .0f);
	// Unit vector:
	p *= (1.0f / P10_norm);

	// each column is a unit vector:
	const CMatrixDouble44 HM = mrpt::math::generateAxisBaseFromDirectionAndAxis(
		p, 2 /* provided vector is "z"*/);

	// Transformation:
	const mrpt::poses::CPose3D T(
		HM.blockCopy<3, 3>(0, 0), mrpt::math::TPoint3D(m_x0, m_y0, m_z0));

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

	ASSERT_ABOVEEQ_(m_headRatio, .0f);
	ASSERT_BELOWEQ_(m_headRatio, 1.0f);

	const float r0 = m_smallRadius, r1 = m_largeRadius,
				h0 = P10_norm * (1.0f - m_headRatio), h1 = P10_norm;

	// cylinder walls:
	for (unsigned int i = 0; i < m_slices; i++)
	{
		const auto ip = (i + 1) % m_slices;

		tris.emplace_back(
			T.composePoint(P3f(r0 * circle[i].x, r0 * circle[i].y, .0f)),
			T.composePoint(P3f(r0 * circle[ip].x, r0 * circle[ip].y, .0f)),
			T.composePoint(P3f(r0 * circle[i].x, r0 * circle[i].y, h0)));

		tris.emplace_back(
			T.composePoint(P3f(r0 * circle[ip].x, r0 * circle[ip].y, .0f)),
			T.composePoint(P3f(r0 * circle[ip].x, r0 * circle[ip].y, h0)),
			T.composePoint(P3f(r0 * circle[i].x, r0 * circle[i].y, h0)));
	}

	// top cone:
	for (unsigned int i = 0; i < m_slices; i++)
	{
		const auto ip = (i + 1) % m_slices;
		tris.emplace_back(
			T.composePoint(P3f(r1 * circle[i].x, r1 * circle[i].y, h0)),
			T.composePoint(P3f(r1 * circle[ip].x, r1 * circle[ip].y, h0)),
			T.composePoint(P3f(.0f, .0f, h1)));
	}

	// All faces, same color:
	for (auto& t : tris) t.setColor(m_color);
}

uint8_t CArrow::serializeGetVersion() const { return 2; }
void CArrow::serializeTo(mrpt::serialization::CArchive& out) const
{
	writeToStreamRender(out);
	out << m_x0 << m_y0 << m_z0;
	out << m_x1 << m_y1 << m_z1;
	out << m_headRatio << m_smallRadius << m_largeRadius;
	out << m_slices;
}

void CArrow::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		case 1:
		case 2:
		{
			readFromStreamRender(in);
			in >> m_x0 >> m_y0 >> m_z0;
			in >> m_x1 >> m_y1 >> m_z1;
			in >> m_headRatio >> m_smallRadius >> m_largeRadius;
			if (version == 1)
			{
				float arrow_roll, arrow_pitch, arrow_yaw;
				in >> arrow_roll >> arrow_pitch >> arrow_yaw;
			}
			if (version >= 2) in >> m_slices;
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
	CRenderizable::notifyChange();
}

void CArrow::serializeTo(mrpt::serialization::CSchemeArchiveBase& out) const
{
	SCHEMA_SERIALIZE_DATATYPE_VERSION(1);
	out["x0"] = m_x0;
	out["y0"] = m_y0;
	out["z0"] = m_z0;
	out["x1"] = m_x1;
	out["y1"] = m_y1;
	out["z1"] = m_z1;
	out["headRatio"] = m_headRatio;
	out["smallRadius"] = m_smallRadius;
	out["largeRadius"] = m_largeRadius;
	out["slices"] = m_slices;
}

void CArrow::serializeFrom(mrpt::serialization::CSchemeArchiveBase& in)
{
	uint8_t version;
	SCHEMA_DESERIALIZE_DATATYPE_VERSION();
	switch (version)
	{
		case 1:
		{
			m_x0 = static_cast<float>(in["x0"]);
			m_y0 = static_cast<float>(in["y0"]);
			m_z0 = static_cast<float>(in["z0"]);
			m_x1 = static_cast<float>(in["x1"]);
			m_y1 = static_cast<float>(in["y1"]);
			m_z1 = static_cast<float>(in["z1"]);
			m_headRatio = static_cast<float>(in["headRatio"]);
			m_smallRadius = static_cast<float>(in["smallRadius"]);
			m_largeRadius = static_cast<float>(in["largeRadius"]);
			m_slices = static_cast<unsigned int>(in["slices"]);
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	}
}
void CArrow::getBoundingBox(
	mrpt::math::TPoint3D& bb_min, mrpt::math::TPoint3D& bb_max) const
{
	bb_min.x = std::min(m_x0, m_x1);
	bb_min.y = std::min(m_y0, m_y1);
	bb_min.z = std::min(m_z0, m_z1);

	bb_max.x = std::max(m_x0, m_x1);
	bb_max.y = std::max(m_y0, m_y1);
	bb_max.z = std::max(m_z0, m_z1);

	// Convert to coordinates of my parent:
	m_pose.composePoint(bb_min, bb_min);
	m_pose.composePoint(bb_max, bb_max);
}
