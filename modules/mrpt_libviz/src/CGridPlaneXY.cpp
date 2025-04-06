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

auto CGridPlaneXY::internalBoundingBoxLocal() const -> mrpt::math::TBoundingBoxf
{
  return mrpt::math::TBoundingBoxf::FromUnsortedPoints({m_xMin, m_yMin, 0}, {m_xMax, m_yMax, 0});
}
