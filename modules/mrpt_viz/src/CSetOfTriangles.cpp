/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2025, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "viz-precomp.h"  // Precompiled header
//
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/viz/CSetOfTriangles.h>

using namespace mrpt;
using namespace mrpt::viz;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CSetOfTriangles, CVisualObject, mrpt::viz)

void CSetOfTriangles::clearTriangles()
{
  std::unique_lock<std::shared_mutex> trisWriteLock(
      VisualObjectParams_Triangles::m_trianglesMtx.data);
  auto& tris = VisualObjectParams_Triangles::m_triangles;
  tris.clear();
  polygonsUpToDate = false;
  CVisualObject::notifyChange();
}

size_t CSetOfTriangles::getTrianglesCount() const
{
  std::shared_lock<std::shared_mutex> trisReadLock(
      VisualObjectParams_Triangles::m_trianglesMtx.data);
  return shaderTrianglesBuffer().size();
}

uint8_t CSetOfTriangles::serializeGetVersion() const { return 0; }
void CSetOfTriangles::serializeTo(mrpt::serialization::CArchive& out) const
{
  writeToStreamRender(out);

  std::shared_lock<std::shared_mutex> trisReadLock(
      VisualObjectParams_Triangles::m_trianglesMtx.data);

  auto n = (uint32_t)shaderTrianglesBuffer().size();
  out << n;
  for (size_t i = 0; i < n; i++) shaderTrianglesBuffer()[i].writeTo(out);
}
void CSetOfTriangles::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  std::unique_lock<std::shared_mutex> trisLck(VisualObjectParams_Triangles::m_trianglesMtx.data);
  auto& tris = VisualObjectParams_Triangles::m_triangles;

  switch (version)
  {
    case 0:
    {
      readFromStreamRender(in);
      uint32_t n;
      in >> n;
      tris.assign(n, TTriangle());
      for (size_t i = 0; i < n; i++) tris[i].readFrom(in);
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };
  polygonsUpToDate = false;
  CVisualObject::notifyChange();
}

bool CSetOfTriangles::traceRay(const mrpt::poses::CPose3D& o, double& dist) const
{
  if (!polygonsUpToDate) updatePolygons();
  return mrpt::math::traceRay(m_polygons, (o - getCPose()).asTPose(), dist);
}
CVisualObject& CSetOfTriangles::setColor_u8(const mrpt::img::TColor& c)
{
  CVisualObject::notifyChange();
  setColor_u8(c);
  std::unique_lock<std::shared_mutex> trisLck(VisualObjectParams_Triangles::m_trianglesMtx.data);
  auto& tris = VisualObjectParams_Triangles::m_triangles;

  for (auto& t : tris) t.setColor(c);
  return *this;
}

CVisualObject& CSetOfTriangles::setColorA_u8(const uint8_t a)
{
  CVisualObject::notifyChange();
  setColorA_u8(a);
  std::unique_lock<std::shared_mutex> trisLck(VisualObjectParams_Triangles::m_trianglesMtx.data);
  auto& tris = VisualObjectParams_Triangles::m_triangles;

  for (auto& t : tris) t.setColor(getColor_u8());
  return *this;
}

void CSetOfTriangles::getPolygons(std::vector<mrpt::math::TPolygon3D>& polys) const
{
  if (!polygonsUpToDate) updatePolygons();
  size_t N = m_polygons.size();
  for (size_t i = 0; i < N; i++) polys[i] = m_polygons[i].poly;
}

void CSetOfTriangles::updatePolygons() const
{
  std::unique_lock<std::shared_mutex> trisLck(VisualObjectParams_Triangles::m_trianglesMtx.data);
  auto& tris = VisualObjectParams_Triangles::m_triangles;

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
  CVisualObject::notifyChange();
}

auto CSetOfTriangles::internalBoundingBoxLocal() const -> mrpt::math::TBoundingBoxf
{
  return trianglesBoundingBox();
}

void CSetOfTriangles::insertTriangles(const CSetOfTriangles::Ptr& p)
{
  ASSERT_(p);

  std::unique_lock<std::shared_mutex> trisLck(VisualObjectParams_Triangles::m_trianglesMtx.data);

  std::shared_lock<std::shared_mutex> trisOtherLck(p->m_trianglesMtx.data);

  auto& tris = VisualObjectParams_Triangles::m_triangles;
  auto& trisOther = p->shaderTrianglesBuffer();

  reserve(tris.size() + trisOther.size());
  tris.insert(tris.end(), trisOther.begin(), trisOther.end());
  polygonsUpToDate = false;
  CVisualObject::notifyChange();
}
