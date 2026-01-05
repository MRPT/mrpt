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

#include <mrpt/serialization/CArchive.h>
#include <mrpt/viz/CSimpleLine.h>

using namespace mrpt;
using namespace mrpt::viz;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CSimpleLine, CVisualObject, mrpt::viz)

CSimpleLine::CSimpleLine(
    float x0,
    float y0,
    float z0,
    float x1,
    float y1,
    float z1,
    float lineWidth,
    bool antiAliasing) :
    m_x0(x0), m_y0(y0), m_z0(z0), m_x1(x1), m_y1(y1), m_z1(z1)
{
  VisualObjectParams_Lines::setLineWidth(lineWidth);
  VisualObjectParams_Lines::enableAntiAliasing(antiAliasing);
}

uint8_t CSimpleLine::serializeGetVersion() const { return 2; }
void CSimpleLine::serializeTo(mrpt::serialization::CArchive& out) const
{
  writeToStreamRender(out);
  out << m_x0 << m_y0 << m_z0;
  out << m_x1 << m_y1 << m_z1;
  VisualObjectParams_Lines::params_serialize(out);  // v2
}

void CSimpleLine::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  switch (version)
  {
    case 1:
    case 2:
    {
      readFromStreamRender(in);
      in >> m_x0 >> m_y0 >> m_z0;
      in >> m_x1 >> m_y1 >> m_z1;
      if (version >= 2)
      {
        VisualObjectParams_Lines::params_deserialize(in);  // v2
      }
      else
      {
        VisualObjectParams_Lines::setLineWidth(in.ReadAs<float>());
        if (version >= 1) VisualObjectParams_Lines::enableAntiAliasing(in.ReadAs<bool>());
      }
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };
  CVisualObject::notifyChange();
}

auto CSimpleLine::internalBoundingBoxLocal() const -> mrpt::math::TBoundingBoxf
{
  return mrpt::math::TBoundingBoxf::FromUnsortedPoints(
      {std::min(m_x0, m_x1), std::min(m_y0, m_y1), std::min(m_z0, m_z1)},
      {std::max(m_x0, m_x1), std::max(m_y0, m_y1), std::max(m_z0, m_z1)});
}
