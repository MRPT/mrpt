/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <mrpt/serialization/CArchive.h>
#include <mrpt/viz/CSetOfTriangles.h>
#include <mrpt/viz/CTexturedPlane.h>

using namespace mrpt;
using namespace mrpt::viz;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CTexturedPlane, CVisualObject, mrpt::viz)

CTexturedPlane::CTexturedPlane(float x_min, float x_max, float y_min, float y_max)
{
  VisualObjectParams_Triangles::enableLight(false);
  VisualObjectParams_TexturedTriangles::enableLight(false);

  setPlaneCorners(x_min, x_max, y_min, y_max);
}
uint8_t CTexturedPlane::serializeGetVersion() const { return 2; }
void CTexturedPlane::serializeTo(mrpt::serialization::CArchive& out) const
{
  writeToStreamRender(out);

  out << m_xMin << m_xMax;
  out << m_yMin << m_yMax;

  writeToStreamTexturedObject(out);
}

void CTexturedPlane::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  switch (version)
  {
    case 0:
    case 1:
      THROW_EXCEPTION("Deserialization of old formats not supported.");
      break;
    case 2:
    {
      readFromStreamRender(in);
      in >> m_xMin >> m_xMax;
      in >> m_yMin >> m_yMax;
      readFromStreamTexturedObject(in);
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };
  CVisualObject::notifyChange();
}

void CTexturedPlane::updateBuffers() const
{
  using P2f = mrpt::math::TPoint2Df;
  using P3f = mrpt::math::TPoint3Df;

  // Populate textured triangles buffer
  {
    auto& tris = VisualObjectParams_TexturedTriangles::m_triangles;
    std::unique_lock<std::shared_mutex> writeLock(
        VisualObjectParams_TexturedTriangles::m_trianglesMtx.data);

    tris.clear();

    const auto col = getColor_u8();

    {
      TTriangle t;
      t.vertices[0].xyzrgba.pt = P3f(m_xMin, m_yMin, 0);
      t.vertices[1].xyzrgba.pt = P3f(m_xMax, m_yMin, 0);
      t.vertices[2].xyzrgba.pt = P3f(m_xMax, m_yMax, 0);

      t.vertices[0].uv = P2f(0, 0);
      t.vertices[1].uv = P2f(1, 0);
      t.vertices[2].uv = P2f(1, 1);

      for (int i = 0; i < 3; i++)
      {
        t.vertices[i].xyzrgba.r = col.R;
        t.vertices[i].xyzrgba.g = col.G;
        t.vertices[i].xyzrgba.b = col.B;
        t.vertices[i].xyzrgba.a = col.A;
      }

      t.computeNormals();
      tris.emplace_back(t);
    }
    {
      TTriangle t;
      t.vertices[0].xyzrgba.pt = P3f(m_xMin, m_yMin, 0);
      t.vertices[1].xyzrgba.pt = P3f(m_xMax, m_yMax, 0);
      t.vertices[2].xyzrgba.pt = P3f(m_xMin, m_yMax, 0);

      t.vertices[0].uv = P2f(0, 0);
      t.vertices[1].uv = P2f(1, 1);
      t.vertices[2].uv = P2f(0, 1);

      for (int i = 0; i < 3; i++)
      {
        t.vertices[i].xyzrgba.r = col.R;
        t.vertices[i].xyzrgba.g = col.G;
        t.vertices[i].xyzrgba.b = col.B;
        t.vertices[i].xyzrgba.a = col.A;
      }

      t.computeNormals();
      tris.emplace_back(t);
    }
  }

  // Populate plain triangles buffer (used when no texture is assigned)
  {
    std::unique_lock<std::shared_mutex> trisWriteLock(
        VisualObjectParams_Triangles::m_trianglesMtx.data);
    auto& tris = VisualObjectParams_Triangles::m_triangles;
    tris.clear();

    const auto col = getColor_u8();
    TTriangle t;
    for (int i = 0; i < 3; i++)
    {
      t.vertices[i].xyzrgba.r = col.R;
      t.vertices[i].xyzrgba.g = col.G;
      t.vertices[i].xyzrgba.b = col.B;
      t.vertices[i].xyzrgba.a = col.A;
    }

    t.vertices[0].xyzrgba.pt = P3f(m_xMin, m_yMin, 0);
    t.vertices[1].xyzrgba.pt = P3f(m_xMax, m_yMin, 0);
    t.vertices[2].xyzrgba.pt = P3f(m_xMax, m_yMax, 0);

    t.computeNormals();
    tris.emplace_back(t);

    t.vertices[0].xyzrgba.pt = P3f(m_xMin, m_yMin, 0);
    t.vertices[1].xyzrgba.pt = P3f(m_xMax, m_yMax, 0);
    t.vertices[2].xyzrgba.pt = P3f(m_xMin, m_yMax, 0);

    t.computeNormals();
    tris.emplace_back(t);
  }
}

bool CTexturedPlane::traceRay(const mrpt::poses::CPose3D& o, double& dist) const
{
  if (!polygonUpToDate) updatePoly();
  return math::traceRay(tmpPoly, (o - getCPose()).asTPose(), dist);
}

void CTexturedPlane::updatePoly() const
{
  TPolygon3D poly(4);
  poly[0].x = poly[1].x = m_xMin;
  poly[2].x = poly[3].x = m_xMax;
  poly[0].y = poly[3].y = m_yMin;
  poly[1].y = poly[2].y = m_yMax;
  for (size_t i = 0; i < 4; i++) poly[i].z = 0;
  tmpPoly.resize(1);
  tmpPoly[0] = poly;
  polygonUpToDate = true;
}

auto CTexturedPlane::internalBoundingBoxLocal() const -> mrpt::math::TBoundingBoxf
{
  return mrpt::math::TBoundingBoxf::FromUnsortedPoints(
      {m_xMin, m_yMin, 0.f}, {m_xMax, m_yMax, 0.f});
}
