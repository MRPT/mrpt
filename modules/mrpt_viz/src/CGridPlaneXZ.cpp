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
#include <mrpt/viz/CGridPlaneXZ.h>

using namespace mrpt;
using namespace mrpt::viz;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CGridPlaneXZ, CVisualObject, mrpt::viz)

/** Constructor */
CGridPlaneXZ::CGridPlaneXZ(
    float xMin,
    float xMax,
    float zMin,
    float zMax,
    float y,
    float frequency,
    float lineWidth,
    bool antiAliasing) :
    m_xMin(xMin), m_xMax(xMax), m_zMin(zMin), m_zMax(zMax), m_plane_y(y), m_frequency(frequency)
{
  setLineWidth(lineWidth);
  enableAntiAliasing(antiAliasing);
}

uint8_t CGridPlaneXZ::serializeGetVersion() const { return 2; }
void CGridPlaneXZ::serializeTo(mrpt::serialization::CArchive& out) const
{
  writeToStreamRender(out);
  out << m_xMin << m_xMax;
  out << m_zMin << m_zMax << m_plane_y;
  out << m_frequency;
  VisualObjectParams_Lines::params_serialize(out);
}

void CGridPlaneXZ::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  switch (version)
  {
    case 0:
    case 1:
    case 2:
    {
      readFromStreamRender(in);
      in >> m_xMin >> m_xMax;
      in >> m_zMin >> m_zMax >> m_plane_y;
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

auto CGridPlaneXZ::internalBoundingBoxLocal() const -> mrpt::math::TBoundingBoxf
{
  return mrpt::math::TBoundingBoxf::FromUnsortedPoints({m_xMin, 0, m_zMin}, {m_xMax, 0, m_zMax});
}
