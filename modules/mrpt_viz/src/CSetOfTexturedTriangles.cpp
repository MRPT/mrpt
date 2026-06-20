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

#include <mrpt/math/geometry.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/viz/CSetOfTexturedTriangles.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::viz;
using namespace mrpt::math;

IMPLEMENTS_SERIALIZABLE(CSetOfTexturedTriangles, CVisualObject, mrpt::viz)

uint8_t CSetOfTexturedTriangles::serializeGetVersion() const { return 2; }
void CSetOfTexturedTriangles::serializeTo(mrpt::serialization::CArchive& out) const
{
  std::shared_lock<std::shared_mutex> readLock(m_trianglesMtx.data);

  uint32_t n;

  writeToStreamRender(out);
  writeToStreamTexturedObject(out);

  n = static_cast<uint32_t>(m_triangles.size());

  out << n;

  for (uint32_t i = 0; i < n; i++) m_triangles[i].writeTo(out);
}

void CSetOfTexturedTriangles::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  std::unique_lock<std::shared_mutex> writeLock(m_trianglesMtx.data);

  switch (version)
  {
    case 0:
    case 1:
    case 2:
    {
      readFromStreamRender(in);
      if (version >= 2)
      {
        readFromStreamTexturedObject(in);
      }
      else
      {  // Old version.
        THROW_EXCEPTION("deserializing old version not supported.");
      }

      uint32_t n;
      in >> n;
      m_triangles.resize(n);

      for (uint32_t i = 0; i < n; i++) m_triangles[i].readFrom(in);
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };
  CVisualObject::notifyChange();
}

bool CSetOfTexturedTriangles::traceRay(const mrpt::poses::CPose3D& o, double& dist) const
{
  std::shared_lock<std::shared_mutex> readLock(m_trianglesMtx.data);

  // Build a list of TPolygonWithPlane from the triangle buffer and delegate
  // to the generic mrpt::math::traceRay implementation.
  const size_t N = m_triangles.size();
  std::vector<mrpt::math::TPolygonWithPlane> polys(N);
  mrpt::math::TPolygon3D tmp(3);
  for (size_t i = 0; i < N; i++)
  {
    const auto& t = m_triangles[i];
    for (size_t j = 0; j < 3; j++)
    {
      tmp[j].x = t.x(j);
      tmp[j].y = t.y(j);
      tmp[j].z = t.z(j);
    }
    polys[i] = tmp;
  }
  return mrpt::math::traceRay(polys, (o - getCPose()).asTPose(), dist);
}

auto CSetOfTexturedTriangles::internalBoundingBoxLocal() const -> mrpt::math::TBoundingBoxf
{
  return trianglesBoundingBox();
}
