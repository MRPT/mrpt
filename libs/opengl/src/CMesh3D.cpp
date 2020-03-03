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

	MRPT_TODO("CONT");
}

void CMesh3D::onUpdateBuffers_Triangles()
{
	auto& tris = CRenderizableShaderTriangles::m_triangles;
	tris.clear();

	const auto &c0 = m_corner_min, &c1 = m_corner_max;
	using P3 = mrpt::math::TPoint3D;
	MRPT_TODO("CONT");
}
void CMesh3D::onUpdateBuffers_Points()
{
	//
	MRPT_TODO("CONT");
}

#if 0
void CMesh3D::render(const RenderContext& rc) const
{
#if MRPT_HAS_OPENGL_GLUT

	glEnable(GL_NORMALIZE);  // So the GPU normalizes the normals instead of
	// doing it in the CPU

	if (m_vertices.size() == 0) return;

	//---------------------------------------------------------------------------------------------------------
	//			Rendering - Test whether changing the rendering mode
	// continuously
	// is
	// very slow (or not)
	//---------------------------------------------------------------------------------------------------------

	// Render the faces
	if (m_showFaces)
	{
		glColor4f(face_color[0], face_color[1], face_color[2], face_color[3]);

		for (unsigned int f = 0; f < m_num_faces; f++)
		{
			// Assign normals to faces (if on)
			if (m_computeNormals)
				glNormal3f(m_normals[f][0], m_normals[f][1], m_normals[f][2]);

			// Render Quads
			if (m_is_quad[f])
			{
				glBegin(GL_QUADS);
				for (int i = 0; i < 4; i++)
				{
					const unsigned int vert_ind = m_face_verts[f][i];
					glVertex3f(
						m_vertices[vert_ind][0], m_vertices[vert_ind][1],
						m_vertices[vert_ind][2]);
				}
				glEnd();
			}
			// Render Triangles
			else
			{
				glBegin(GL_TRIANGLES);
				for (int i = 0; i < 3; i++)
				{
					const unsigned int vert_ind = m_face_verts[f][i];
					glVertex3f(
						m_vertices[vert_ind][0], m_vertices[vert_ind][1],
						m_vertices[vert_ind][2]);
				}
				glEnd();
			}
		}
	}

	// Render the edges - They are rendered twice, which is redundant but simple
	if (m_showEdges)
	{
		glColor4f(edge_color[0], edge_color[1], edge_color[2], edge_color[3]);
		glDisable(GL_LIGHTING);  //??
		glLineWidth(m_lineWidth);
		glEnable(GL_LINE_SMOOTH);
		glBegin(GL_LINES);
		for (unsigned int f = 0; f < m_num_faces; f++)
		{
			const unsigned char num_vert = 3 + m_is_quad[f];
			for (int i = 0; i < num_vert - 1; i++)
			{
				const unsigned int v_0 = m_face_verts[f][i];
				const unsigned int v_1 = m_face_verts[f][i + 1];

				glVertex3f(
					m_vertices[v_0][0], m_vertices[v_0][1], m_vertices[v_0][2]);
				glVertex3f(
					m_vertices[v_1][0], m_vertices[v_1][1], m_vertices[v_1][2]);
			}

			// The last vertex of the face needs to be connected to the first as
			// well
			const int v_0 = m_face_verts[f][num_vert - 1];
			const int v_1 = m_face_verts[f][0];

			glVertex3f(
				m_vertices[v_0][0], m_vertices[v_0][1], m_vertices[v_0][2]);
			glVertex3f(
				m_vertices[v_1][0], m_vertices[v_1][1], m_vertices[v_1][2]);
		}
		glEnd();
		glDisable(GL_LINE_SMOOTH);
	}

	// Render the vertices
	if (m_showVertices)
	{
		glColor4f(vert_color[0], vert_color[1], vert_color[2], vert_color[3]);
		glPointSize(m_pointSize);
		glEnable(GL_POINT_SMOOTH);
		glBegin(GL_POINTS);
		for (unsigned int v = 0; v < m_vertices.size(); v++)
			glVertex3f(m_vertices[v][0], m_vertices[v][1], m_vertices[v][2]);

		glEnd();
		glDisable(GL_POINT_SMOOTH);
	}

#endif
}
#endif

uint8_t CMesh3D::serializeGetVersion() const { return 0; }
void CMesh3D::serializeTo(mrpt::serialization::CArchive& out) const
{
	writeToStreamRender(out);
	out << m_showEdges << m_showFaces << m_showVertices << m_computeNormals;
	out << m_is_quad << m_face_verts << m_vertices << m_normals;
}

void CMesh3D::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
	readFromStreamRender(in);
	THROW_EXCEPTION("write me");
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

void CMesh3D::setEdgeColor(float r, float g, float b, float a)
{
	edge_color[0] = r;
	edge_color[1] = g;
	edge_color[2] = b;
	edge_color[3] = a;
}

void CMesh3D::setFaceColor(float r, float g, float b, float a)
{
	face_color[0] = r;
	face_color[1] = g;
	face_color[2] = b;
	face_color[3] = a;
}

void CMesh3D::setVertColor(float r, float g, float b, float a)
{
	vert_color[0] = r;
	vert_color[1] = g;
	vert_color[2] = b;
	vert_color[3] = a;
}
