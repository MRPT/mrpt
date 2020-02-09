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
#if MRPT_HAS_OPENGL_GLUT

	// Vertices and colors are already stored into m_triangles.

	// Eval normals:
	const auto n = m_triangles.size();
	m_trianglesNormals.resize(n);
	for (size_t i = 0; i < n; i++)
	{
		const auto& t = m_triangles[i];
		const float ax = t.x(1) - t.x(0);
		const float ay = t.y(1) - t.y(0);
		const float az = t.z(1) - t.z(0);
		const float bx = t.x(2) - t.x(0);
		const float by = t.y(2) - t.y(0);
		const float bz = t.z(2) - t.z(0);

		auto& no = m_trianglesNormals[i];
		no.x = ay * bz - az * by;
		no.y = -ax * bz + az * bx;
		no.z = ax * by - ay * bx;
	}

	// Define OpenGL buffers:
	m_trianglesBuffer = mrpt::opengl::make_buffer(
		GL_ARRAY_BUFFER, m_triangles.data(), sizeof(m_triangles[0]) * n);

	m_normalsBuffer = mrpt::opengl::make_buffer(
		GL_ARRAY_BUFFER, m_trianglesNormals.data(),
		sizeof(m_trianglesNormals[0]) * n);

	// Generate a name for a new array.
	glGenVertexArrays(1, &m_vao);
	// Make the new array active, creating it if necessary.
	glBindVertexArray(m_vao);

#endif
}

void CSetOfTriangles::render(
	const mrpt::opengl::TRenderMatrices& state,
	mrpt::opengl::Program& shaders) const
{
#if MRPT_HAS_OPENGL_GLUT

	if (m_enableTransparency)
	{
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	}
	else
	{
		glEnable(GL_DEPTH_TEST);
		glDisable(GL_BLEND);
	}
	CHECK_OPENGL_ERROR();

	// Set up the vertex array:
	const GLint attr_position = shaders.attributeId("position");
	glEnableVertexAttribArray(attr_position);
	glBindBuffer(GL_ARRAY_BUFFER, m_trianglesBuffer);
	glVertexAttribPointer(
		attr_position, /* attribute */
		3, /* size */
		GL_FLOAT, /* type */
		GL_FALSE, /* normalized? */
		sizeof(mrpt::math::TPointXYZRGBAf), /* stride */
		BUFFER_OFFSET(offsetof(mrpt::math::TPointXYZRGBAf, pt.x)));
	CHECK_OPENGL_ERROR();

	// Set up the color array:
	const GLint attr_color = shaders.attributeId("vertexColor");
	glEnableVertexAttribArray(attr_color);
	glBindBuffer(GL_ARRAY_BUFFER, m_trianglesBuffer);
	glVertexAttribPointer(
		attr_color, /* attribute */
		4, /* size */
		GL_FLOAT, /* type */
		GL_FALSE, /* normalized? */
		sizeof(mrpt::math::TPointXYZRGBAf), /* stride */
		BUFFER_OFFSET(offsetof(mrpt::math::TPointXYZRGBAf, R)));
	CHECK_OPENGL_ERROR();

	MRPT_TODO("Handle normals!");

	glDrawArrays(GL_TRIANGLES, 0, 3 * m_triangles.size());
	CHECK_OPENGL_ERROR();

	glDisableVertexAttribArray(attr_position);
	glDisableVertexAttribArray(attr_color);

	if (m_enableTransparency) glDisable(GL_BLEND);
#endif
}

uint8_t CSetOfTriangles::serializeGetVersion() const { return 1; }
void CSetOfTriangles::serializeTo(mrpt::serialization::CArchive& out) const
{
	writeToStreamRender(out);
	auto n = (uint32_t)m_triangles.size();
	out << n;
	for (size_t i = 0; i < n; i++) m_triangles[i].writeTo(out);

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
			for (size_t i = 0; i < n; i++) m_triangles[i].readFrom(in);

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
	return mrpt::math::traceRay(m_polygons, (o - this->m_pose).asTPose(), dist);
}
CRenderizable& CSetOfTriangles::setColor_u8(const mrpt::img::TColor& c)
{
	CRenderizable::notifyChange();
	m_color = c;
	mrpt::img::TColorf col(c);
	for (auto& m_triangle : m_triangles)
		for (size_t i = 0; i < 3; i++)
		{
			m_triangle.r(i) = col.R;
			m_triangle.g(i) = col.G;
			m_triangle.b(i) = col.B;
			m_triangle.a(i) = col.A;
		}
	return *this;
}

CRenderizable& CSetOfTriangles::setColorR_u8(const uint8_t r)
{
	CRenderizable::notifyChange();
	m_color.R = r;
	const float col = r / 255.f;
	for (auto& m_triangle : m_triangles)
		for (size_t i = 0; i < 3; i++) m_triangle.r(i) = col;
	return *this;
}

CRenderizable& CSetOfTriangles::setColorG_u8(const uint8_t g)
{
	CRenderizable::notifyChange();
	m_color.G = g;
	const float col = g / 255.f;
	for (auto& m_triangle : m_triangles)
		for (size_t i = 0; i < 3; i++) m_triangle.g(i) = col;
	return *this;
}

CRenderizable& CSetOfTriangles::setColorB_u8(const uint8_t b)
{
	CRenderizable::notifyChange();
	m_color.B = b;
	const float col = b / 255.f;
	for (auto& m_triangle : m_triangles)
		for (size_t i = 0; i < 3; i++) m_triangle.b(i) = col;
	return *this;
}

CRenderizable& CSetOfTriangles::setColorA_u8(const uint8_t a)
{
	CRenderizable::notifyChange();
	m_color.A = a;
	const float col = a / 255.f;
	for (auto& m_triangle : m_triangles)
		for (size_t i = 0; i < 3; i++) m_triangle.a(i) = col;
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
