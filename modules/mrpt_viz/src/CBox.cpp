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
#include <mrpt/viz/CBox.h>

using namespace mrpt;
using namespace mrpt::viz;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CBox, CVisualObject, mrpt::viz)

CBox::CBox(
    const mrpt::math::TPoint3D& corner1,
    const mrpt::math::TPoint3D& corner2,
    bool is_wireframe,
    float lineWidth) :
    m_wireframe(is_wireframe)
{
  VisualObjectParams_Lines::setLineWidth(lineWidth);
  setBoxCorners(corner1, corner2);
}

uint8_t CBox::serializeGetVersion() const { return 3; }
void CBox::serializeTo(mrpt::serialization::CArchive& out) const
{
  writeToStreamRender(out);
  // version 0
  out << m_corner_min.x << m_corner_min.y << m_corner_min.z << m_corner_max.x << m_corner_max.y
      << m_corner_max.z << m_wireframe;
  // Version 1:
  out << m_draw_border << m_solidborder_color;
  VisualObjectParams_Triangles::params_serialize(out);  // v2
  VisualObjectParams_Lines::params_serialize(out);      // v3
}

void CBox::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  switch (version)
  {
    case 0:
    case 1:
    case 2:
    case 3:
    {
      readFromStreamRender(in);
      in >> m_corner_min.x >> m_corner_min.y >> m_corner_min.z >> m_corner_max.x >>
          m_corner_max.y >> m_corner_max.z >> m_wireframe;
      // Version 1:
      if (version >= 1)
        in >> m_draw_border >> m_solidborder_color;
      else
      {
        m_draw_border = false;
      }
      if (version >= 2) VisualObjectParams_Triangles::params_deserialize(in);
      if (version >= 3) VisualObjectParams_Lines::params_deserialize(in);
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };
  CVisualObject::notifyChange();
}

void CBox::setBoxCorners(const mrpt::math::TPoint3D& corner1, const mrpt::math::TPoint3D& corner2)
{
  CVisualObject::notifyChange();

  // Order the coordinates so we always have the min/max in their right
  // position:
  m_corner_min.x = std::min(corner1.x, corner2.x);
  m_corner_min.y = std::min(corner1.y, corner2.y);
  m_corner_min.z = std::min(corner1.z, corner2.z);

  m_corner_max.x = std::max(corner1.x, corner2.x);
  m_corner_max.y = std::max(corner1.y, corner2.y);
  m_corner_max.z = std::max(corner1.z, corner2.z);
}

bool CBox::traceRay(
    [[maybe_unused]] const mrpt::poses::CPose3D& o, [[maybe_unused]] double& dist) const
{
  THROW_EXCEPTION("TO DO");
}

auto CBox::internalBoundingBoxLocal() const -> mrpt::math::TBoundingBoxf
{
  return mrpt::math::TBoundingBoxf::FromUnsortedPoints(
      m_corner_min.cast<float>(), m_corner_max.cast<float>());
}
