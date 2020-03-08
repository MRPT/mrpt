/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header

#include <mrpt/img/color_maps.h>
#include <mrpt/opengl/CMesh3D.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/stl_serialization.h>

#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CMesh3D, CRenderizable, mrpt::opengl)

CMesh3D::~CMesh3D() = default;

void CMesh3D::loadMesh(
	unsigned int num_verts, unsigned int num_faces, int* verts_per_face,
	int* face_verts, float* vert_coords)
{
	// Fill number of vertices for each face
	m_is_quad.resize(num_faces);
	for (unsigned int i = 0; i < num_faces; i++)
	{
		if (verts_per_face[i] == 3)
			m_is_quad[i] = false;
		else if (verts_per_face[i] == 4)
			m_is_quad[i] = true;
		else
		{
			THROW_EXCEPTION(
				"Incorrect mesh format. It can only be composed of triangles "
				"and/or quads");
		}
	}

	// Fill the vertices of each face
	m_face_verts.resize(num_faces);
	unsigned int count = 0;
	for (unsigned int f = 0; f < num_faces; f++)
	{
		m_face_verts[f][0] = face_verts[count++];
		m_face_verts[f][1] = face_verts[count++];
		m_face_verts[f][2] = face_verts[count++];
		if (m_is_quad[f])
			m_face_verts[f][3] = face_verts[count++];
		else
			m_face_verts[f][3] = -1;  // Meaning it is a triangle
	}

	// Fill the 3D coordinates of the vertex
	m_vertices.resize(num_verts);
	for (unsigned int i = 0; i < num_verts; i++)
	{
		m_vertices[i][0] = vert_coords[3 * i];
		m_vertices[i][1] = vert_coords[3 * i + 1];
		m_vertices[i][2] = vert_coords[3 * i + 2];
	}

	// Compute the mesh normals (if on)
	if (m_computeNormals)
	{
		m_normals.resize(num_faces);

		for (unsigned int f = 0; f < num_faces; f++)
		{
			const unsigned int v1 = m_face_verts[f][0];
			const unsigned int v2 = m_face_verts[f][1];
			const unsigned int v3 = m_face_verts[f][2];
			const unsigned int v4 = m_face_verts[f][3];

			if (m_is_quad[f])
			{
				const float vec1[3] = {m_vertices[v3][0] - m_vertices[v1][0],
									   m_vertices[v3][1] - m_vertices[v1][1],
									   m_vertices[v3][2] - m_vertices[v1][2]};
				const float vec2[3] = {m_vertices[v4][0] - m_vertices[v2][0],
									   m_vertices[v4][1] - m_vertices[v2][1],
									   m_vertices[v4][2] - m_vertices[v2][2]};
				m_normals[f][0] = vec1[1] * vec2[2] - vec1[2] * vec2[1];
				m_normals[f][1] = vec1[2] * vec2[0] - vec1[0] * vec2[2];
				m_normals[f][2] = vec1[0] * vec2[1] - vec1[1] * vec2[0];
			}
			else
			{
				const float vec1[3] = {m_vertices[v2][0] - m_vertices[v1][0],
									   m_vertices[v2][1] - m_vertices[v1][1],
									   m_vertices[v2][2] - m_vertices[v1][2]};
				const float vec2[3] = {m_vertices[v3][0] - m_vertices[v1][0],
									   m_vertices[v3][1] - m_vertices[v1][1],
									   m_vertices[v3][2] - m_vertices[v1][2]};
				m_normals[f][0] = vec1[1] * vec2[2] - vec1[2] * vec2[1];
				m_normals[f][1] = vec1[2] * vec2[0] - vec1[0] * vec2[2];
				m_normals[f][2] = vec1[0] * vec2[1] - vec1[1] * vec2[0];
			}
			m_normals[f].unitarize();
		}
	}

	CRenderizable::notifyChange();
}

void CMesh3D::loadMesh(
	unsigned int num_verts, unsigned int num_faces,
	const mrpt::math::CMatrixDynamic<bool>& is_quad,
	const mrpt::math::CMatrixDynamic<int>& face_verts,
	const mrpt::math::CMatrixDynamic<float>& vert_coords)
{
	// Fill number of vertices for each face
	m_is_quad.resize(num_faces);
	for (unsigned int i = 0; i < num_faces; i++) m_is_quad[i] = is_quad(i, 0);

	// Fill the vertices of each face
	m_face_verts.resize(num_faces);
	for (unsigned int f = 0; f < num_faces; f++)
	{
		m_face_verts[f][0] = face_verts(0, f);
		m_face_verts[f][1] = face_verts(1, f);
		m_face_verts[f][2] = face_verts(2, f);
		if (m_is_quad[f])
			m_face_verts[f][3] = face_verts(3, f);
		else
			m_face_verts[f][3] = -1;  // Meaning it is a triangle
	}

	// Fill the 3D coordinates of the vertex
	m_vertices.resize(num_verts);
	for (unsigned int i = 0; i < num_verts; i++)
	{
		m_vertices[i][0] = vert_coords(0, i);
		m_vertices[i][1] = vert_coords(1, i);
		m_vertices[i][2] = vert_coords(2, i);
	}

	// Compute the mesh normals (if on)
	m_normals.resize(num_faces);
	if (m_computeNormals)
		for (unsigned int f = 0; f < num_faces; f++)
		{
			const unsigned int v1 = m_face_verts[f][0];
			const unsigned int v2 = m_face_verts[f][1];
			const unsigned int v3 = m_face_verts[f][2];
			const unsigned int v4 = m_face_verts[f][3];

			if (m_is_quad[f])
			{
				const float vec1[3] = {m_vertices[v3][0] - m_vertices[v1][0],
									   m_vertices[v3][1] - m_vertices[v1][1],
									   m_vertices[v3][2] - m_vertices[v1][2]};
				const float vec2[3] = {m_vertices[v4][0] - m_vertices[v2][0],
									   m_vertices[v4][1] - m_vertices[v2][1],
									   m_vertices[v4][2] - m_vertices[v2][2]};
				m_normals[f][0] = vec1[1] * vec2[2] - vec1[2] * vec2[1];
				m_normals[f][1] = vec1[2] * vec2[0] - vec1[0] * vec2[2];
				m_normals[f][2] = vec1[0] * vec2[1] - vec1[1] * vec2[0];
			}
			else
			{
				const float vec1[3] = {m_vertices[v2][0] - m_vertices[v1][0],
									   m_vertices[v2][1] - m_vertices[v1][1],
									   m_vertices[v2][2] - m_vertices[v1][2]};
				const float vec2[3] = {m_vertices[v3][0] - m_vertices[v1][0],
									   m_vertices[v3][1] - m_vertices[v1][1],
									   m_vertices[v3][2] - m_vertices[v1][2]};
				m_normals[f][0] = vec1[1] * vec2[2] - vec1[2] * vec2[1];
				m_normals[f][1] = vec1[2] * vec2[0] - vec1[0] * vec2[2];
				m_normals[f][2] = vec1[0] * vec2[1] - vec1[1] * vec2[0];
			}
		}

	CRenderizable::notifyChange();
}

void CMesh3D::render(const RenderContext& rc) const
{
	switch (rc.shader_id)
	{
		case DefaultShaderID::TRIANGLES:
			if (m_showFaces) CRenderizableShaderTriangles::render(rc);
			break;
		case DefaultShaderID::WIREFRAME:
			if (m_showEdges) CRenderizableShaderWireFrame::render(rc);
			break;
		case DefaultShaderID::POINTS:
			if (m_showVertices) CRenderizableShaderWireFrame::render(rc);
			break;
	};
}
void CMesh3D::renderUpdateBuffers() const
{
	CRenderizableShaderPoints::renderUpdateBuffers();
	CRenderizableShaderTriangles::renderUpdateBuffers();
	CRenderizableShaderWireFrame::renderUpdateBuffers();
}

void CMesh3D::onUpdateBuffers_Wireframe()
{
	auto& vbd = CRenderizableShaderWireFrame::m_vertex_buffer_data;
	auto& cbd = CRenderizableShaderWireFrame::m_color_buffer_data;
	vbd.clear();

	for (unsigned int f = 0; f < m_face_verts.size(); f++)
	{
		const unsigned char num_vert = 3 + m_is_quad[f];
		for (int i = 0; i < num_vert - 1; i++)
		{
			const unsigned int v_0 = m_face_verts[f][i];
			const unsigned int v_1 = m_face_verts[f][i + 1];

			vbd.emplace_back(m_vertices[v_0]);
			vbd.emplace_back(m_vertices[v_1]);
		}

		// The last vertex of the face needs to be connected to the first as
		// well
		const int v_0 = m_face_verts[f][num_vert - 1];
		const int v_1 = m_face_verts[f][0];

		vbd.emplace_back(m_vertices[v_0]);
		vbd.emplace_back(m_vertices[v_1]);
	}

	cbd.assign(vbd.size(), edge_color.asTColor());
}

void CMesh3D::onUpdateBuffers_Triangles()
{
	auto& tris = CRenderizableShaderTriangles::m_triangles;
	tris.clear();

	for (unsigned int f = 0; f < m_is_quad.size(); f++)
	{
		// Assign normals to faces (if on)
		const auto& normal = m_normals[f];

		tris.emplace_back(
			m_vertices[m_face_verts[f][0]], m_vertices[m_face_verts[f][1]],
			m_vertices[m_face_verts[f][2]], normal, normal, normal);
		if (m_is_quad[f])
		{
			tris.emplace_back(
				m_vertices[m_face_verts[f][0]], m_vertices[m_face_verts[f][2]],
				m_vertices[m_face_verts[f][3]], normal, normal, normal);
		}
	}

	for (auto& t : tris) t.setColor(face_color);
}
void CMesh3D::onUpdateBuffers_Points()
{
	auto& vbd = CRenderizableShaderPoints::m_vertex_buffer_data;
	auto& cbd = CRenderizableShaderPoints::m_color_buffer_data;

	vbd = m_vertices;
	cbd.assign(m_vertices.size(), vert_color.asTColor());
}

uint8_t CMesh3D::serializeGetVersion() const { return 0; }
void CMesh3D::serializeTo(mrpt::serialization::CArchive& out) const
{
	writeToStreamRender(out);
	out << m_showEdges << m_showFaces << m_showVertices << m_computeNormals;
	out << m_is_quad << m_vertices << m_normals;
	out.WriteAs<uint32_t>(m_face_verts.size());
	if (!m_face_verts.empty())
		out.WriteBufferFixEndianness<uint32_t>(
			m_face_verts[0].data(),
			m_face_verts.size() * m_face_verts[0].size());
}

void CMesh3D::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
	readFromStreamRender(in);
	in >> m_showEdges >> m_showFaces >> m_showVertices >> m_computeNormals;
	in >> m_is_quad >> m_vertices >> m_normals;
	const auto N = in.ReadAs<uint32_t>();
	m_face_verts.resize(N);
	if (!m_face_verts.empty())
		in.ReadBufferFixEndianness<uint32_t>(
			m_face_verts[0].data(),
			m_face_verts.size() * m_face_verts[0].size());
}

void CMesh3D::getBoundingBox(
	mrpt::math::TPoint3D& bb_min, mrpt::math::TPoint3D& bb_max) const
{
	if (m_vertices.empty())
	{
		bb_max = TPoint3D(0, 0, 0);
		bb_min = TPoint3D(0, 0, 0);
	}
	else
	{
		bb_min.x = std::numeric_limits<double>::max();
		bb_min.y = std::numeric_limits<double>::max();
		bb_min.z = std::numeric_limits<double>::max();
		bb_max.x = -std::numeric_limits<double>::max();
		bb_max.y = -std::numeric_limits<double>::max();
		bb_max.z = -std::numeric_limits<double>::max();

		for (unsigned int i = 0; i < m_vertices.size(); i++)
		{
			// Max
			mrpt::keep_max(bb_max.x, m_vertices[i][0]);
			mrpt::keep_max(bb_max.y, m_vertices[i][1]);
			mrpt::keep_max(bb_max.z, m_vertices[i][2]);

			// Min
			mrpt::keep_min(bb_min.x, m_vertices[i][0]);
			mrpt::keep_min(bb_min.y, m_vertices[i][1]);
			mrpt::keep_min(bb_min.z, m_vertices[i][2]);
		}
	}

	// Convert to coordinates of my parent:
	m_pose.composePoint(bb_min, bb_min);
	m_pose.composePoint(bb_max, bb_max);
}
