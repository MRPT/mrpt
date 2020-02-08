/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/opengl/CSetOfTriangles.h>
#include <mrpt/serialization/CArchive.h>
#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CSetOfTriangles, CRenderizable, mrpt::opengl)

void CSetOfTriangles::renderUpdateBuffers() const
{
	//
	MRPT_TODO("Implement me!");
}

void CSetOfTriangles::render(const mrpt::opengl::TRenderMatrices& state, mrpt::opengl::Program& shaders) const
{
#if MRPT_HAS_OPENGL_GLUT

	if (m_enableTransparency)
	{
		// glDisable(GL_DEPTH_TEST);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	}
	else
	{
		glEnable(GL_DEPTH_TEST);
		glDisable(GL_BLEND);
	}

	vector<TTriangle>::const_iterator it;

	glEnable(GL_NORMALIZE);  // Normalize normals
	glBegin(GL_TRIANGLES);

	for (it = m_triangles.begin(); it != m_triangles.end(); ++it)
	{
		// Compute the normal vector:
		// ---------------------------------
		float ax = it->x[1] - it->x[0];
		float ay = it->y[1] - it->y[0];
		float az = it->z[1] - it->z[0];

		float bx = it->x[2] - it->x[0];
		float by = it->y[2] - it->y[0];
		float bz = it->z[2] - it->z[0];

		glNormal3f(ay * bz - az * by, -ax * bz + az * bx, ax * by - ay * bx);

		glColor4f(it->r[0], it->g[0], it->b[0], it->a[0]);
		glVertex3f(it->x[0], it->y[0], it->z[0]);

		glColor4f(it->r[1], it->g[1], it->b[1], it->a[1]);
		glVertex3f(it->x[1], it->y[1], it->z[1]);

		glColor4f(it->r[2], it->g[2], it->b[2], it->a[2]);
		glVertex3f(it->x[2], it->y[2], it->z[2]);
	}

	glEnd();
	glDisable(GL_NORMALIZE);

	glDisable(GL_BLEND);
#endif
}

static void triangle_writeToStream(
	mrpt::serialization::CArchive& o, const mrpt::opengl::TTriangle& t)
{
	o.WriteBufferFixEndianness(t.x, 3);
	o.WriteBufferFixEndianness(t.y, 3);
	o.WriteBufferFixEndianness(t.z, 3);

	o.WriteBufferFixEndianness(t.r, 3);
	o.WriteBufferFixEndianness(t.g, 3);
	o.WriteBufferFixEndianness(t.b, 3);
	o.WriteBufferFixEndianness(t.a, 3);
}
static void triangle_readFromStream(
	mrpt::serialization::CArchive& i, mrpt::opengl::TTriangle& t)
{
	i.ReadBufferFixEndianness(t.x, 3);
	i.ReadBufferFixEndianness(t.y, 3);
	i.ReadBufferFixEndianness(t.z, 3);

	i.ReadBufferFixEndianness(t.r, 3);
	i.ReadBufferFixEndianness(t.g, 3);
	i.ReadBufferFixEndianness(t.b, 3);
	i.ReadBufferFixEndianness(t.a, 3);
}

uint8_t CSetOfTriangles::serializeGetVersion() const { return 1; }
void CSetOfTriangles::serializeTo(mrpt::serialization::CArchive& out) const
{
	writeToStreamRender(out);
	auto n = (uint32_t)m_triangles.size();
	out << n;
	for (size_t i = 0; i < n; i++) triangle_writeToStream(out, m_triangles[i]);

	// Version 1:
	out << m_enableTransparency;
}
void CSetOfTriangles::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		case 1:
		{
			readFromStreamRender(in);
			uint32_t n;
			in >> n;
			m_triangles.assign(n, TTriangle());
			for (size_t i = 0; i < n; i++)
				triangle_readFromStream(in, m_triangles[i]);

			if (version >= 1)
				in >> m_enableTransparency;
			else
				m_enableTransparency = true;
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
	polygonsUpToDate = false;
	CRenderizable::notifyChange();
}

bool CSetOfTriangles::traceRay(
	const mrpt::poses::CPose3D& o, double& dist) const
{
	if (!polygonsUpToDate) updatePolygons();
	return mrpt::math::traceRay(
		tmpPolygons, (o - this->m_pose).asTPose(), dist);
}
CRenderizable& CSetOfTriangles::setColor_u8(const mrpt::img::TColor& c)
{
	CRenderizable::notifyChange();
	m_color = c;
	mrpt::img::TColorf col(c);
	for (auto& m_triangle : m_triangles)
		for (size_t i = 0; i < 3; i++)
		{
			m_triangle.r[i] = col.R;
			m_triangle.g[i] = col.G;
			m_triangle.b[i] = col.B;
			m_triangle.a[i] = col.A;
		}
	return *this;
}

CRenderizable& CSetOfTriangles::setColorR_u8(const uint8_t r)
{
	CRenderizable::notifyChange();
	m_color.R = r;
	const float col = r / 255.f;
	for (auto& m_triangle : m_triangles)
		for (size_t i = 0; i < 3; i++) m_triangle.r[i] = col;
	return *this;
}

CRenderizable& CSetOfTriangles::setColorG_u8(const uint8_t g)
{
	CRenderizable::notifyChange();
	m_color.G = g;
	const float col = g / 255.f;
	for (auto& m_triangle : m_triangles)
		for (size_t i = 0; i < 3; i++) m_triangle.g[i] = col;
	return *this;
}

CRenderizable& CSetOfTriangles::setColorB_u8(const uint8_t b)
{
	CRenderizable::notifyChange();
	m_color.B = b;
	const float col = b / 255.f;
	for (auto& m_triangle : m_triangles)
		for (size_t i = 0; i < 3; i++) m_triangle.b[i] = col;
	return *this;
}

CRenderizable& CSetOfTriangles::setColorA_u8(const uint8_t a)
{
	CRenderizable::notifyChange();
	m_color.A = a;
	const float col = a / 255.f;
	for (auto& m_triangle : m_triangles)
		for (size_t i = 0; i < 3; i++) m_triangle.a[i] = col;
	return *this;
}

void CSetOfTriangles::getPolygons(
	std::vector<mrpt::math::TPolygon3D>& polys) const
{
	if (!polygonsUpToDate) updatePolygons();
	size_t N = tmpPolygons.size();
	for (size_t i = 0; i < N; i++) polys[i] = tmpPolygons[i].poly;
}

void CSetOfTriangles::updatePolygons() const
{
	TPolygon3D tmp(3);
	size_t N = m_triangles.size();
	tmpPolygons.resize(N);
	for (size_t i = 0; i < N; i++)
		for (size_t j = 0; j < 3; j++)
		{
			const TTriangle& t = m_triangles[i];
			tmp[j].x = t.x[j];
			tmp[j].y = t.y[j];
			tmp[j].z = t.z[j];
			tmpPolygons[i] = tmp;
		}
	polygonsUpToDate = true;
	CRenderizable::notifyChange();
}

void CSetOfTriangles::getBoundingBox(
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
		keep_min(bb_min.x, t.x[0]);
		keep_max(bb_max.x, t.x[0]);
		keep_min(bb_min.y, t.y[0]);
		keep_max(bb_max.y, t.y[0]);
		keep_min(bb_min.z, t.z[0]);
		keep_max(bb_max.z, t.z[0]);

		keep_min(bb_min.x, t.x[1]);
		keep_max(bb_max.x, t.x[1]);
		keep_min(bb_min.y, t.y[1]);
		keep_max(bb_max.y, t.y[1]);
		keep_min(bb_min.z, t.z[1]);
		keep_max(bb_max.z, t.z[1]);

		keep_min(bb_min.x, t.x[2]);
		keep_max(bb_max.x, t.x[2]);
		keep_min(bb_min.y, t.y[2]);
		keep_max(bb_max.y, t.y[2]);
		keep_min(bb_min.z, t.z[2]);
		keep_max(bb_max.z, t.z[2]);
	}

	// Convert to coordinates of my parent:
	m_pose.composePoint(bb_min, bb_min);
	m_pose.composePoint(bb_max, bb_max);
}

void CSetOfTriangles::insertTriangles(const CSetOfTriangles::Ptr& p)
{
	reserve(m_triangles.size() + p->m_triangles.size());
	m_triangles.insert(
		m_triangles.end(), p->m_triangles.begin(), p->m_triangles.end());
	polygonsUpToDate = false;
	CRenderizable::notifyChange();
}
