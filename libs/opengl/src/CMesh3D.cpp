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

#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CMesh3D, CRenderizable, mrpt::opengl)

CMesh3D::CMesh3D(
	bool enableTransparency, bool antiAliasing, bool enableShowEdges,
	bool enableShowFaces, bool enableShowVertices)
	: m_enableTransparency(enableTransparency),
	  m_antiAliasing(antiAliasing),
	  m_showEdges(enableShowEdges),
	  m_showFaces(enableShowFaces),
	  m_showVertices(enableShowVertices)
{
	m_color.R = 1.f;
	m_color.G = 0.f;
	m_color.B = 0.f;
	m_color.A = 1.f;
}

CMesh3D::~CMesh3D() = default;

void CMesh3D::loadMesh(
	unsigned int num_verts, unsigned int num_faces, int* verts_per_face,
	int* face_verts, float* vert_coords)
{
	m_num_verts = num_verts;
	m_num_faces = num_faces;

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
			printf(
				"\n Incorrect mesh format. It can only be composed of "
				"triangles and/or quads.");
			return;
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
	m_vert_coords.resize(num_verts);
	for (unsigned int i = 0; i < num_verts; i++)
	{
		m_vert_coords[i][0] = vert_coords[3 * i];
		m_vert_coords[i][1] = vert_coords[3 * i + 1];
		m_vert_coords[i][2] = vert_coords[3 * i + 2];
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
				const float vec1[3] = {
					m_vert_coords[v3][0] - m_vert_coords[v1][0],
					m_vert_coords[v3][1] - m_vert_coords[v1][1],
					m_vert_coords[v3][2] - m_vert_coords[v1][2]};
				const float vec2[3] = {
					m_vert_coords[v4][0] - m_vert_coords[v2][0],
					m_vert_coords[v4][1] - m_vert_coords[v2][1],
					m_vert_coords[v4][2] - m_vert_coords[v2][2]};
				m_normals[f][0] = vec1[1] * vec2[2] - vec1[2] * vec2[1];
				m_normals[f][1] = vec1[2] * vec2[0] - vec1[0] * vec2[2];
				m_normals[f][2] = vec1[0] * vec2[1] - vec1[1] * vec2[0];
			}
			else
			{
				const float vec1[3] = {
					m_vert_coords[v2][0] - m_vert_coords[v1][0],
					m_vert_coords[v2][1] - m_vert_coords[v1][1],
					m_vert_coords[v2][2] - m_vert_coords[v1][2]};
				const float vec2[3] = {
					m_vert_coords[v3][0] - m_vert_coords[v1][0],
					m_vert_coords[v3][1] - m_vert_coords[v1][1],
					m_vert_coords[v3][2] - m_vert_coords[v1][2]};
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
	MRPT_TODO("Refactor: one STL container of faces & another vertices");
	m_num_verts = num_verts;
	m_num_faces = num_faces;

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
	m_vert_coords.resize(num_verts);
	for (unsigned int i = 0; i < num_verts; i++)
	{
		m_vert_coords[i][0] = vert_coords(0, i);
		m_vert_coords[i][1] = vert_coords(1, i);
		m_vert_coords[i][2] = vert_coords(2, i);
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
				const float vec1[3] = {
					m_vert_coords[v3][0] - m_vert_coords[v1][0],
					m_vert_coords[v3][1] - m_vert_coords[v1][1],
					m_vert_coords[v3][2] - m_vert_coords[v1][2]};
				const float vec2[3] = {
					m_vert_coords[v4][0] - m_vert_coords[v2][0],
					m_vert_coords[v4][1] - m_vert_coords[v2][1],
					m_vert_coords[v4][2] - m_vert_coords[v2][2]};
				m_normals[f][0] = vec1[1] * vec2[2] - vec1[2] * vec2[1];
				m_normals[f][1] = vec1[2] * vec2[0] - vec1[0] * vec2[2];
				m_normals[f][2] = vec1[0] * vec2[1] - vec1[1] * vec2[0];
			}
			else
			{
				const float vec1[3] = {
					m_vert_coords[v2][0] - m_vert_coords[v1][0],
					m_vert_coords[v2][1] - m_vert_coords[v1][1],
					m_vert_coords[v2][2] - m_vert_coords[v1][2]};
				const float vec2[3] = {
					m_vert_coords[v3][0] - m_vert_coords[v1][0],
					m_vert_coords[v3][1] - m_vert_coords[v1][1],
					m_vert_coords[v3][2] - m_vert_coords[v1][2]};
				m_normals[f][0] = vec1[1] * vec2[2] - vec1[2] * vec2[1];
				m_normals[f][1] = vec1[2] * vec2[0] - vec1[0] * vec2[2];
				m_normals[f][2] = vec1[0] * vec2[1] - vec1[1] * vec2[0];
			}
		}

	CRenderizable::notifyChange();
}

void CMesh3D::renderUpdateBuffers() const
{
	//
	MRPT_TODO("Implement me!");
}

void CMesh3D::render() const
{
#if MRPT_HAS_OPENGL_GLUT

	if (m_enableTransparency || m_antiAliasing)
	{
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	}
	else
	{
		glEnable(GL_DEPTH_TEST);
		glDisable(GL_BLEND);
	}

	glEnable(GL_NORMALIZE);  // So the GPU normalizes the normals instead of
	// doing it in the CPU

	if (m_num_verts == 0) return;

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
						m_vert_coords[vert_ind][0], m_vert_coords[vert_ind][1],
						m_vert_coords[vert_ind][2]);
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
						m_vert_coords[vert_ind][0], m_vert_coords[vert_ind][1],
						m_vert_coords[vert_ind][2]);
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
					m_vert_coords[v_0][0], m_vert_coords[v_0][1],
					m_vert_coords[v_0][2]);
				glVertex3f(
					m_vert_coords[v_1][0], m_vert_coords[v_1][1],
					m_vert_coords[v_1][2]);
			}

			// The last vertex of the face needs to be connected to the first as
			// well
			const int v_0 = m_face_verts[f][num_vert - 1];
			const int v_1 = m_face_verts[f][0];

			glVertex3f(
				m_vert_coords[v_0][0], m_vert_coords[v_0][1],
				m_vert_coords[v_0][2]);
			glVertex3f(
				m_vert_coords[v_1][0], m_vert_coords[v_1][1],
				m_vert_coords[v_1][2]);
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
		for (unsigned int v = 0; v < m_num_verts; v++)
			glVertex3f(
				m_vert_coords[v][0], m_vert_coords[v][1], m_vert_coords[v][2]);

		glEnd();
		glDisable(GL_POINT_SMOOTH);
	}

#endif
}

uint8_t CMesh3D::serializeGetVersion() const { return 0; }
void CMesh3D::serializeTo(mrpt::serialization::CArchive& out) const
{
	//********** To do **********
	THROW_EXCEPTION("not implemented yet!");

	//	writeToStreamRender(out);

	//	// Version 0:
	//	out <<	m_enableTransparency;
	//	out <<	m_showEdges;
	//	out <<	m_showFaces;
	//	out <<	m_showVertices;
	//	out << 	m_computeNormals;
	//	out << 	m_num_verts;
	//	out << 	m_num_faces;

	//	bool			*m_is_quad;
	//	f_verts		*m_face_verts;
	//	coord3D		*m_vert_coords;
	//	coord3D		*m_normals;
}

void CMesh3D::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
	//********** To do ************
}

void CMesh3D::getBoundingBox(
	mrpt::math::TPoint3D& bb_min, mrpt::math::TPoint3D& bb_max) const
{
	// Extreme initialization
	bb_min.x = 10000.f;
	bb_min.y = 10000.f;
	bb_min.z = 10000.f;
	bb_max.x = -10000.f;
	bb_max.y = -10000.f;
	bb_max.z = -10000.f;

	if (m_num_verts == 0)
		printf(
			"\n The mesh is empty and has no size. The returned information "
			"has no meaning.");
	else
	{
		for (unsigned int i = 0; i < m_num_verts; i++)
		{
			// Max
			if (m_vert_coords[i][0] > bb_max.x) bb_max.x = m_vert_coords[i][0];
			if (m_vert_coords[i][1] > bb_max.y) bb_max.y = m_vert_coords[i][1];
			if (m_vert_coords[i][2] > bb_max.z) bb_max.z = m_vert_coords[i][2];

			// Min
			if (m_vert_coords[i][0] < bb_min.x) bb_min.x = m_vert_coords[i][0];
			if (m_vert_coords[i][1] < bb_min.y) bb_min.y = m_vert_coords[i][1];
			if (m_vert_coords[i][2] < bb_min.z) bb_min.z = m_vert_coords[i][2];
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
