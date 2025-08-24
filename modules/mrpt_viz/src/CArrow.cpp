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
#include <mrpt/math/geometry.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSchemeArchiveBase.h>
#include <mrpt/viz/CArrow.h>

#include <memory>

using namespace mrpt;
using namespace mrpt::viz;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CArrow, CVisualObject, mrpt::viz)

uint8_t CArrow::serializeGetVersion() const { return 3; }
void CArrow::serializeTo(mrpt::serialization::CArchive& out) const
{
  writeToStreamRender(out);
  out << m_x0 << m_y0 << m_z0;
  out << m_x1 << m_y1 << m_z1;
  out << m_headRatio << m_smallRadius << m_largeRadius;
  out << m_slices;
  VisualObjectParams_Triangles::params_serialize(out);  // v3
}

void CArrow::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  switch (version)
  {
    case 0:
    case 1:
    case 2:
    case 3:
    {
      readFromStreamRender(in);
      in >> m_x0 >> m_y0 >> m_z0;
      in >> m_x1 >> m_y1 >> m_z1;
      in >> m_headRatio >> m_smallRadius >> m_largeRadius;
      if (version == 1)
      {
        float arrow_roll, arrow_pitch, arrow_yaw;
        in >> arrow_roll >> arrow_pitch >> arrow_yaw;
      }
      if (version >= 2) in >> m_slices;
      if (version >= 3) VisualObjectParams_Triangles::params_deserialize(in);
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };
  CVisualObject::notifyChange();
}

void CArrow::serializeTo(mrpt::serialization::CSchemeArchiveBase& out) const
{
  SCHEMA_SERIALIZE_DATATYPE_VERSION(1);
  out["x0"] = m_x0;
  out["y0"] = m_y0;
  out["z0"] = m_z0;
  out["x1"] = m_x1;
  out["y1"] = m_y1;
  out["z1"] = m_z1;
  out["headRatio"] = m_headRatio;
  out["smallRadius"] = m_smallRadius;
  out["largeRadius"] = m_largeRadius;
  out["slices"] = m_slices;
}

void CArrow::serializeFrom(mrpt::serialization::CSchemeArchiveBase& in)
{
  uint8_t version;
  SCHEMA_DESERIALIZE_DATATYPE_VERSION();
  switch (version)
  {
    case 1:
    {
      m_x0 = static_cast<float>(in["x0"]);
      m_y0 = static_cast<float>(in["y0"]);
      m_z0 = static_cast<float>(in["z0"]);
      m_x1 = static_cast<float>(in["x1"]);
      m_y1 = static_cast<float>(in["y1"]);
      m_z1 = static_cast<float>(in["z1"]);
      m_headRatio = static_cast<float>(in["headRatio"]);
      m_smallRadius = static_cast<float>(in["smallRadius"]);
      m_largeRadius = static_cast<float>(in["largeRadius"]);
      m_slices = static_cast<unsigned int>(in["slices"]);
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  }
}
auto CArrow::internalBoundingBoxLocal() const -> mrpt::math::TBoundingBoxf
{
  return mrpt::math::TBoundingBoxf::FromUnsortedPoints(
      {std::min(m_x0, m_x1), std::min(m_y0, m_y1), std::min(m_z0, m_z1)},
      {std::max(m_x0, m_x1), std::max(m_y0, m_y1), std::max(m_z0, m_z1)});
}
