/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"	 // Precompiled header
//
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/opengl/CSetOfTriangles.h>
#include <mrpt/opengl/opengl_api.h>
#include <mrpt/serialization/CArchive.h>

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

void CSetOfTriangles::clearTriangles()
{
	std::unique_lock<std::shared_mutex> trisWriteLock(
		CRenderizableShaderTriangles::m_trianglesMtx.data);
	auto& tris = CRenderizableShaderTriangles::m_triangles;
	tris.clear();
	polygonsUpToDate = false;
	CRenderizable::notifyChange();
}

size_t CSetOfTriangles::getTrianglesCount() const
{
	std::shared_lock<std::shared_mutex> trisReadLock(
		CRenderizableShaderTriangles::m_trianglesMtx.data);
	return shaderTrianglesBuffer().size();
}

uint8_t CSetOfTriangles::serializeGetVersion() const { return 0; }
void CSetOfTriangles::serializeTo(mrpt::serialization::CArchive& out) const
{
	writeToStreamRender(out);

	std::shared_lock<std::shared_mutex> trisReadLock(
		CRenderizableShaderTriangles::m_trianglesMtx.data);

	auto n = (uint32_t)shaderTrianglesBuffer().size();
	out << n;
	for (size_t i = 0; i < n; i++)
		shaderTrianglesBuffer()[i].writeTo(out);
}
void CSetOfTriangles::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	std::unique_lock<std::shared_mutex> trisLck(
		CRenderizableShaderTriangles::m_trianglesMtx.data);
	auto& tris = CRenderizableShaderTriangles::m_triangles;

	switch (version)
	{
		case 0:
		{
			readFromStreamRender(in);
			uint32_t n;
			in >> n;
			tris.assign(n, TTriangle());
			for (size_t i = 0; i < n; i++)
				tris[i].readFrom(in);
		}
		break;
		default: MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
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
	std::unique_lock<std::shared_mutex> trisLck(
		CRenderizableShaderTriangles::m_trianglesMtx.data);
	auto& tris = CRenderizableShaderTriangles::m_triangles;

	m_color = c;
	for (auto& t : tris)
		t.setColor(c);
	return *this;
}

CRenderizable& CSetOfTriangles::setColorR_u8(const uint8_t r)
{
	CRenderizable::notifyChange();
	std::unique_lock<std::shared_mutex> trisLck(
		CRenderizableShaderTriangles::m_trianglesMtx.data);
	auto& tris = CRenderizableShaderTriangles::m_triangles;
	m_color.R = r;
	for (auto& t : tris)
		t.setColor(m_color);
	return *this;
}

CRenderizable& CSetOfTriangles::setColorG_u8(const uint8_t g)
{
	CRenderizable::notifyChange();
	std::unique_lock<std::shared_mutex> trisLck(
		CRenderizableShaderTriangles::m_trianglesMtx.data);
	auto& tris = CRenderizableShaderTriangles::m_triangles;
	m_color.G = g;
	for (auto& t : tris)
		t.setColor(m_color);
	return *this;
}

CRenderizable& CSetOfTriangles::setColorB_u8(const uint8_t b)
{
	CRenderizable::notifyChange();
	std::unique_lock<std::shared_mutex> trisLck(
		CRenderizableShaderTriangles::m_trianglesMtx.data);
	auto& tris = CRenderizableShaderTriangles::m_triangles;
	m_color.B = b;
	for (auto& t : tris)
		t.setColor(m_color);
	return *this;
}

CRenderizable& CSetOfTriangles::setColorA_u8(const uint8_t a)
{
	CRenderizable::notifyChange();
	std::unique_lock<std::shared_mutex> trisLck(
		CRenderizableShaderTriangles::m_trianglesMtx.data);
	auto& tris = CRenderizableShaderTriangles::m_triangles;
	m_color.A = a;
	for (auto& t : tris)
		t.setColor(m_color);
	return *this;
}

void CSetOfTriangles::getPolygons(
	std::vector<mrpt::math::TPolygon3D>& polys) const
{
	if (!polygonsUpToDate) updatePolygons();
	size_t N = m_polygons.size();
	for (size_t i = 0; i < N; i++)
		polys[i] = m_polygons[i].poly;
}

void CSetOfTriangles::updatePolygons() const
{
	std::unique_lock<std::shared_mutex> trisLck(
		CRenderizableShaderTriangles::m_trianglesMtx.data);
	auto& tris = CRenderizableShaderTriangles::m_triangles;

	TPolygon3D tmp(3);
	size_t N = tris.size();
	m_polygons.resize(N);
	for (size_t i = 0; i < N; i++)
		for (size_t j = 0; j < 3; j++)
		{
			const TTriangle& t = tris[i];
			tmp[j].x = t.x(j);
			tmp[j].y = t.y(j);
			tmp[j].z = t.z(j);
			m_polygons[i] = tmp;
		}
	polygonsUpToDate = true;
	CRenderizable::notifyChange();
}

auto CSetOfTriangles::internalBoundingBoxLocal() const
	-> mrpt::math::TBoundingBoxf
{
	return trianglesBoundingBox();
}

void CSetOfTriangles::insertTriangles(const CSetOfTriangles::Ptr& p)
{
	ASSERT_(p);

	std::unique_lock<std::shared_mutex> trisLck(
		CRenderizableShaderTriangles::m_trianglesMtx.data);

	std::shared_lock<std::shared_mutex> trisOtherLck(p->m_trianglesMtx.data);

	auto& tris = CRenderizableShaderTriangles::m_triangles;
	auto& trisOther = p->shaderTrianglesBuffer();

	reserve(tris.size() + trisOther.size());
	tris.insert(tris.end(), trisOther.begin(), trisOther.end());
	polygonsUpToDate = false;
	CRenderizable::notifyChange();
}
