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

#include "maps-precomp.h"  // Precomp header
//
#include <mrpt/serialization/CArchive.h>
#include <mrpt/viz/CPlanarLaserScan.h>

using namespace mrpt::viz;
using namespace mrpt::math;

IMPLEMENTS_SERIALIZABLE(CPlanarLaserScan, CVisualObject, mrpt::viz)

void CPlanarLaserScan::clear()
{
  CVisualObject::notifyChange();
  m_scan.resizeScan(0);
}

uint8_t CPlanarLaserScan::serializeGetVersion() const { return 3; }
void CPlanarLaserScan::serializeTo(mrpt::serialization::CArchive& out) const
{
  writeToStreamRender(out);
  out << m_scan;
  out << m_line_R << m_line_G << m_line_B << m_line_A << m_points_R << m_points_G << m_points_B
      << m_points_A << m_plane_R << m_plane_G << m_plane_B << m_plane_A << m_enable_points
      << m_enable_line << m_enable_surface;             // new in v1
  VisualObjectParams_Triangles::params_serialize(out);  // v3
}

void CPlanarLaserScan::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  switch (version)
  {
    case 0:
    case 1:
    case 2:
    case 3:
    {
      readFromStreamRender(in);
      in >> m_scan;

      if (version < 2)
      {  //  m_line_width
        float dummy;
        in >> dummy;
      }

      in >> m_line_R >> m_line_G >> m_line_B >> m_line_A;

      if (version < 2)
      {  // m_points_width
        float dummy;
        in >> dummy;
      }
      in >> m_points_R >> m_points_G >> m_points_B >> m_points_A >> m_plane_R >> m_plane_G >>
          m_plane_B >> m_plane_A;

      if (version >= 1)
      {
        in >> m_enable_points >> m_enable_line >> m_enable_surface;  // new in v1
      }
      else
      {
        m_enable_points = m_enable_line = m_enable_surface = true;
      }
      if (version >= 3) VisualObjectParams_Triangles::params_deserialize(in);
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };
}

auto CPlanarLaserScan::internalBoundingBoxLocal() const -> mrpt::math::TBoundingBoxf
{
  // Load into cache:
  if (!m_cache_valid)
  {
    m_cache_valid = true;
    m_cache_points.clear();
    m_cache_points.insertionOptions.minDistBetweenLaserPoints = 0;
    m_cache_points.insertionOptions.isPlanarMap = false;

    m_cache_points.insertObservation(m_scan);
  }

  if (m_cache_points.empty()) return {};

  return m_cache_points.boundingBox();
}

mrpt::math::TPoint3Df CPlanarLaserScan::getLocalRepresentativePoint() const
{
  return {d2f(m_scan.sensorPose.x()), d2f(m_scan.sensorPose.y()), d2f(m_scan.sensorPose.z())};
}
