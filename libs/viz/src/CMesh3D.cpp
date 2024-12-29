/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "viz-precomp.h"  // Precompiled header
//
#include <mrpt/img/color_maps.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/stl_serialization.h>
#include <mrpt/viz/CMesh3D.h>

using namespace mrpt;
using namespace mrpt::viz;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CMesh3D, CVisualObject, mrpt::viz)

CMesh3D::~CMesh3D() = default;

void CMesh3D::loadMesh(
    unsigned int num_verts,
    unsigned int num_faces,
    int* verts_per_face,
    int* face_verts,
    float* vert_coords)
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
      const unsigned int v1 = m_face_verts[f][3];
      const unsigned int v2 = m_face_verts[f][2];
      const unsigned int v3 = m_face_verts[f][1];
      const unsigned int v4 = m_face_verts[f][0];

      if (m_is_quad[f])
      {
        const float vec1[3] = {
            m_vertices[v3][0] - m_vertices[v1][0], m_vertices[v3][1] - m_vertices[v1][1],
            m_vertices[v3][2] - m_vertices[v1][2]};
        const float vec2[3] = {
            m_vertices[v4][0] - m_vertices[v2][0], m_vertices[v4][1] - m_vertices[v2][1],
            m_vertices[v4][2] - m_vertices[v2][2]};
        m_normals[f][0] = vec1[1] * vec2[2] - vec1[2] * vec2[1];
        m_normals[f][1] = vec1[2] * vec2[0] - vec1[0] * vec2[2];
        m_normals[f][2] = vec1[0] * vec2[1] - vec1[1] * vec2[0];
      }
      else
      {
        const float vec1[3] = {
            m_vertices[v2][0] - m_vertices[v1][0], m_vertices[v2][1] - m_vertices[v1][1],
            m_vertices[v2][2] - m_vertices[v1][2]};
        const float vec2[3] = {
            m_vertices[v3][0] - m_vertices[v1][0], m_vertices[v3][1] - m_vertices[v1][1],
            m_vertices[v3][2] - m_vertices[v1][2]};
        m_normals[f][0] = vec1[1] * vec2[2] - vec1[2] * vec2[1];
        m_normals[f][1] = vec1[2] * vec2[0] - vec1[0] * vec2[2];
        m_normals[f][2] = vec1[0] * vec2[1] - vec1[1] * vec2[0];
      }
      m_normals[f] = m_normals[f].unitarize();
    }
  }

  CVisualObject::notifyChange();
}

void CMesh3D::loadMesh(
    unsigned int num_verts,
    unsigned int num_faces,
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
        const float vec1[3] = {
            m_vertices[v3][0] - m_vertices[v1][0], m_vertices[v3][1] - m_vertices[v1][1],
            m_vertices[v3][2] - m_vertices[v1][2]};
        const float vec2[3] = {
            m_vertices[v4][0] - m_vertices[v2][0], m_vertices[v4][1] - m_vertices[v2][1],
            m_vertices[v4][2] - m_vertices[v2][2]};
        m_normals[f][0] = vec1[1] * vec2[2] - vec1[2] * vec2[1];
        m_normals[f][1] = vec1[2] * vec2[0] - vec1[0] * vec2[2];
        m_normals[f][2] = vec1[0] * vec2[1] - vec1[1] * vec2[0];
      }
      else
      {
        const float vec1[3] = {
            m_vertices[v2][0] - m_vertices[v1][0], m_vertices[v2][1] - m_vertices[v1][1],
            m_vertices[v2][2] - m_vertices[v1][2]};
        const float vec2[3] = {
            m_vertices[v3][0] - m_vertices[v1][0], m_vertices[v3][1] - m_vertices[v1][1],
            m_vertices[v3][2] - m_vertices[v1][2]};
        m_normals[f][0] = vec1[1] * vec2[2] - vec1[2] * vec2[1];
        m_normals[f][1] = vec1[2] * vec2[0] - vec1[0] * vec2[2];
        m_normals[f][2] = vec1[0] * vec2[1] - vec1[1] * vec2[0];
      }
    }

  CVisualObject::notifyChange();
}

uint8_t CMesh3D::serializeGetVersion() const { return 1; }
void CMesh3D::serializeTo(mrpt::serialization::CArchive& out) const
{
  writeToStreamRender(out);
  out << m_showEdges << m_showFaces << m_showVertices << m_computeNormals;
  out << m_is_quad << m_vertices << m_normals;
  out.WriteAs<uint32_t>(m_face_verts.size());
  if (!m_face_verts.empty())
    out.WriteBufferFixEndianness<uint32_t>(
        m_face_verts[0].data(), m_face_verts.size() * m_face_verts[0].size());
  VisualObjectParams_Triangles::params_serialize(out);  // v1
}

void CMesh3D::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  switch (version)
  {
    case 0:
    case 1:
    {
      readFromStreamRender(in);
      in >> m_showEdges >> m_showFaces >> m_showVertices >> m_computeNormals;
      in >> m_is_quad >> m_vertices >> m_normals;
      const auto N = in.ReadAs<uint32_t>();
      m_face_verts.resize(N);
      if (!m_face_verts.empty())
        in.ReadBufferFixEndianness<uint32_t>(
            m_face_verts[0].data(), m_face_verts.size() * m_face_verts[0].size());

      if (version >= 1) VisualObjectParams_Triangles::params_deserialize(in);
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };
  CVisualObject::notifyChange();
}

auto CMesh3D::internalBoundingBoxLocal() const -> mrpt::math::TBoundingBoxf
{
  return trianglesBoundingBox();
}
