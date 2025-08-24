/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include "viz-precomp.h"  // Precompiled header
//
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
