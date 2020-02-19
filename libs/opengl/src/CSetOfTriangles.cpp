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

IMPLEMENTS_SERIALIZABLE(
	CSetOfTriangles, CRenderizableShaderTriangles, mrpt::opengl)

void CSetOfTriangles::onUpdateBuffers_Triangles()
{
	// Nothing else to do, data is directly kept up-to-date in base class
	// buffers.
}

uint8_t CSetOfTriangles::serializeGetVersion() const { return 0; }
void CSetOfTriangles::serializeTo(mrpt::serialization::CArchive& out) const
{
	writeToStreamRender(out);
	auto n = (uint32_t)m_triangles.size();
	out << n;
	for (size_t i = 0; i < n; i++) m_triangles[i].writeTo(out);
}
void CSetOfTriangles::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		{
			readFromStreamRender(in);
			uint32_t n;
			in >> n;
			m_triangles.assign(n, TTriangle());
			for (size_t i = 0; i < n; i++) m_triangles[i].readFrom(in);
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
	return mrpt::math::traceRay(m_polygons, (o - this->m_pose).asTPose(), dist);
}
CRenderizable& CSetOfTriangles::setColor_u8(const mrpt::img::TColor& c)
{
	CRenderizable::notifyChange();
	m_color = c;
	for (auto& t : m_triangles) t.setColor(c);
	return *this;
}

CRenderizable& CSetOfTriangles::setColorR_u8(const uint8_t r)
{
	CRenderizable::notifyChange();
	m_color.R = r;
	for (auto& t : m_triangles) t.setColor(m_color);
	return *this;
}

CRenderizable& CSetOfTriangles::setColorG_u8(const uint8_t g)
{
	CRenderizable::notifyChange();
	m_color.G = g;
	for (auto& t : m_triangles) t.setColor(m_color);
	return *this;
}

CRenderizable& CSetOfTriangles::setColorB_u8(const uint8_t b)
{
	CRenderizable::notifyChange();
	m_color.B = b;
	for (auto& t : m_triangles) t.setColor(m_color);
	return *this;
}

CRenderizable& CSetOfTriangles::setColorA_u8(const uint8_t a)
{
	CRenderizable::notifyChange();
	m_color.A = a;
	for (auto& t : m_triangles) t.setColor(m_color);
	return *this;
}

void CSetOfTriangles::getPolygons(
	std::vector<mrpt::math::TPolygon3D>& polys) const
{
	if (!polygonsUpToDate) updatePolygons();
	size_t N = m_polygons.size();
	for (size_t i = 0; i < N; i++) polys[i] = m_polygons[i].poly;
}

void CSetOfTriangles::updatePolygons() const
{
	TPolygon3D tmp(3);
	size_t N = m_triangles.size();
	m_polygons.resize(N);
	for (size_t i = 0; i < N; i++)
		for (size_t j = 0; j < 3; j++)
		{
			const TTriangle& t = m_triangles[i];
			tmp[j].x = t.x(j);
			tmp[j].y = t.y(j);
			tmp[j].z = t.z(j);
			m_polygons[i] = tmp;
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
		keep_min(bb_min.x, t.x(0));
		keep_max(bb_max.x, t.x(0));
		keep_min(bb_min.y, t.y(0));
		keep_max(bb_max.y, t.y(0));
		keep_min(bb_min.z, t.z(0));
		keep_max(bb_max.z, t.z(0));

		keep_min(bb_min.x, t.x(1));
		keep_max(bb_max.x, t.x(1));
		keep_min(bb_min.y, t.y(1));
		keep_max(bb_max.y, t.y(1));
		keep_min(bb_min.z, t.z(1));
		keep_max(bb_max.z, t.z(1));

		keep_min(bb_min.x, t.x(2));
		keep_max(bb_max.x, t.x(2));
		keep_min(bb_min.y, t.y(2));
		keep_max(bb_max.y, t.y(2));
		keep_min(bb_min.z, t.z(2));
		keep_max(bb_max.z, t.z(2));
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
