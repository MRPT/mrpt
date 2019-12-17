/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/math/CMatrixF.h>
#include <mrpt/math/geometry.h>
#include <mrpt/opengl/CArrow.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSchemeArchiveBase.h>

#include <memory>

#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CArrow, CRenderizable, mrpt::opengl)

void CArrow::renderUpdateBuffers() const
{
	//
	MRPT_TODO("Implement me!");
}

void CArrow::render() const
{
#if MRPT_HAS_OPENGL_GLUT

	GLUquadricObj* obj1 = gluNewQuadric();
	GLUquadricObj* obj2 = gluNewQuadric();

	GLfloat mat[16];

	// Compute the direction vector, which will become the transformed z-axis:
	float vx = m_x1 - m_x0;
	float vy = m_y1 - m_y0;
	float vz = m_z1 - m_z0;
	if ((m_arrow_roll != -1.0f) || (m_arrow_pitch != -1.0f) ||
		(m_arrow_yaw != -1.0f))
	{
		m_x0 = 0.0f;
		m_x1 = 0.0f;
		m_y0 = 0.0f;
		m_y1 = 0.1f;
		m_z0 = 0.0f;
		m_z1 = 0.0f;

		float cr = cos(m_arrow_roll);
		float sr = sin(m_arrow_roll);
		float cp = cos(m_arrow_pitch);
		float sp = sin(m_arrow_pitch);
		float cy = cos(m_arrow_yaw);
		float sy = sin(m_arrow_yaw);

		CMatrixFloat m(3, 3), xx(3, 1), out(1, 3);
		m(0, 0) = cr * cp;
		m(0, 1) = cr * sp * sy - sr * cy;
		m(0, 2) = sr * sy + cr * sp * cy;
		m(1, 0) = sr * cp;
		m(1, 1) = sr * sp * sy + cr * cy;
		m(1, 2) = sr * sp * cy - cr * sy;
		m(2, 0) = -sp;
		m(2, 1) = cp * sy;
		m(2, 2) = cp * cy;
		xx(0, 0) = 0.0f;
		xx(1, 0) = 1.0f;
		xx(2, 0) = 0.0f;

		out = m * xx;
		vx = out(0, 0);
		vy = out(1, 0);
		vz = out(2, 0);
	}

	// Normalize:
	const float v_mod = sqrt(square(vx) + square(vy) + square(vz));
	if (v_mod > 0)
	{
		vx /= v_mod;
		vy /= v_mod;
		vz /= v_mod;
	}

	//  A homogeneous transformation matrix, in this order:
	//
	//     0  4  8  12
	//     1  5  9  13
	//     2  6  10 14
	//     3  7  11 15
	//

	mat[3] = mat[7] = mat[11] = 0;
	mat[15] = 1;
	mat[12] = m_x0;
	mat[13] = m_y0;
	mat[14] = m_z0;

	// New Z-axis
	mat[8] = vx;
	mat[9] = vy;
	mat[10] = vz;

	// New X-axis: Perp. to Z
	if (vx != 0 || vy != 0)
	{
		mat[0] = -vy;
		mat[1] = vx;
		mat[2] = 0;
	}
	else
	{
		mat[0] = 0;
		mat[1] = vz;
		mat[2] = -vy;
	}

	// New Y-axis: Perp. to both: the cross product:
	//  | i  j  k |     | i  j  k |
	//  | x0 y0 z0| --> | 8  9  10|
	//  | x1 y1 z1|     | 0  1  2 |
	GLfloat* out_v3 = mat + 4;
	math::crossProduct3D(
		mat + 8,  // 1st vector
		mat + 0,  // 2nd vector
		out_v3  // Output cross product
	);

	glPushMatrix();

	glMultMatrixf(mat);
	// Scale Z to the size of the cylinder:
	glScalef(1.0f, 1.0f, v_mod * (1.0f - m_headRatio));
	gluCylinder(obj1, m_smallRadius, m_smallRadius, 1, 10, 1);

	glPopMatrix();

	// Draw the head of the arrow: a cone (built from a cylinder)
	//-------------------------------------------------------------
	mat[12] = m_x0 + vx * v_mod * (1.0f - m_headRatio);
	mat[13] = m_y0 + vy * v_mod * (1.0f - m_headRatio);
	mat[14] = m_z0 + vz * v_mod * (1.0f - m_headRatio);

	glPushMatrix();

	glMultMatrixf(mat);
	// Scale Z to the size of the cylinder:
	glScalef(1.0f, 1.0f, v_mod * m_headRatio);

	gluCylinder(obj2, m_largeRadius, 0, 1, 10, 10);

	glPopMatrix();

	gluDeleteQuadric(obj1);
	gluDeleteQuadric(obj2);

#endif
}

uint8_t CArrow::serializeGetVersion() const { return 1; }
void CArrow::serializeTo(mrpt::serialization::CArchive& out) const
{
	writeToStreamRender(out);
	out << m_x0 << m_y0 << m_z0;
	out << m_x1 << m_y1 << m_z1;
	out << m_headRatio << m_smallRadius << m_largeRadius;
	out << m_arrow_roll << m_arrow_pitch << m_arrow_yaw;
}

void CArrow::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		{
			readFromStreamRender(in);
			in >> m_x0 >> m_y0 >> m_z0;
			in >> m_x1 >> m_y1 >> m_z1;
			in >> m_headRatio >> m_smallRadius >> m_largeRadius;
		}
		break;
		case 1:
		{
			readFromStreamRender(in);
			in >> m_x0 >> m_y0 >> m_z0;
			in >> m_x1 >> m_y1 >> m_z1;
			in >> m_headRatio >> m_smallRadius >> m_largeRadius;
			in >> m_arrow_roll >> m_arrow_pitch >> m_arrow_yaw;
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
