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
#include <mrpt/viz/CGridPlaneXY.h>

using namespace mrpt;
using namespace mrpt::viz;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CGridPlaneXY, CVisualObject, mrpt::viz)

/** Constructor  */
CGridPlaneXY::CGridPlaneXY(
    float xMin,
    float xMax,
    float yMin,
    float yMax,
    float z,
    float frequency,
    float lineWidth,
    bool antiAliasing) :
    m_xMin(xMin), m_xMax(xMax), m_yMin(yMin), m_yMax(yMax), m_plane_z(z), m_frequency(frequency)
{
  setLineWidth(lineWidth);
  enableAntiAliasing(antiAliasing);
}

uint8_t CGridPlaneXY::serializeGetVersion() const { return 2; }
void CGridPlaneXY::serializeTo(mrpt::serialization::CArchive& out) const
{
  writeToStreamRender(out);
  out << m_xMin << m_xMax;
  out << m_yMin << m_yMax << m_plane_z;
  out << m_frequency;
  VisualObjectParams_Lines::params_serialize(out);
}

void CGridPlaneXY::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  switch (version)
  {
    case 0:
    case 1:
    case 2:
    {
      readFromStreamRender(in);
      in >> m_xMin >> m_xMax;
      in >> m_yMin >> m_yMax >> m_plane_z;
      in >> m_frequency;
      if (version == 1)
      {
        setLineWidth(in.ReadAs<float>());
        enableAntiAliasing(in.ReadAs<bool>());
      }
      else if (version >= 2)
      {
        VisualObjectParams_Lines::params_deserialize(in);
      }
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };
  CVisualObject::notifyChange();
}

void CGridPlaneXY::updateBuffers() const
{
  ASSERT_GT_(m_frequency, 0);

  std::unique_lock<std::shared_mutex> lck(VisualObjectParams_Lines::m_linesMtx.data);

  auto& vbd = VisualObjectParams_Lines::m_vertex_buffer_data;
  auto& cbd = VisualObjectParams_Lines::m_color_buffer_data;
  vbd.clear();
  cbd.clear();

  for (float y = m_yMin; y <= m_yMax; y += m_frequency)
  {
    vbd.emplace_back(m_xMin, y, m_plane_z);
    vbd.emplace_back(m_xMax, y, m_plane_z);
  }

  for (float x = m_xMin; x <= m_xMax; x += m_frequency)
  {
    vbd.emplace_back(x, m_yMin, m_plane_z);
    vbd.emplace_back(x, m_yMax, m_plane_z);
  }
  // The same color to all vertices:
  cbd.assign(vbd.size(), getColor_u8());
}

auto CGridPlaneXY::internalBoundingBoxLocal() const -> mrpt::math::TBoundingBoxf
{
  return mrpt::math::TBoundingBoxf::FromUnsortedPoints({m_xMin, m_yMin, 0}, {m_xMax, m_yMax, 0});
}
