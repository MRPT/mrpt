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
#include <mrpt/viz/CPlanarLaserScan.h>

using namespace mrpt::viz;
using namespace mrpt::math;

IMPLEMENTS_SERIALIZABLE(CPlanarLaserScan, CVisualObject, mrpt::viz)

CPlanarLaserScan::CPlanarLaserScan() { castShadows(false); }

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

void CPlanarLaserScan::updateBuffers() const
{
  // Load scan into cache:
  if (!m_cache_valid)
  {
    m_cache_valid = true;
    m_cache_points.clear();
    m_cache_points.insertionOptions.minDistBetweenLaserPoints = 0;
    m_cache_points.insertionOptions.isPlanarMap = false;

    m_cache_points.insertObservation(m_scan);
  }

  const auto& x = m_cache_points.getPointsBufferRef_x();
  const auto& y = m_cache_points.getPointsBufferRef_y();
  const auto& z = m_cache_points.getPointsBufferRef_z();
  const size_t n = m_cache_points.size();

  // Lines (wireframe along scan endpoints)
  {
    std::unique_lock<std::shared_mutex> lck(VisualObjectParams_Lines::m_linesMtx.data);
    auto& vbd = VisualObjectParams_Lines::m_vertex_buffer_data;
    auto& cbd = VisualObjectParams_Lines::m_color_buffer_data;
    vbd.clear();
    cbd.clear();

    if (m_enable_line && n > 1)
    {
      for (size_t i = 0; i < n - 1; i++)
      {
        vbd.emplace_back(x[i], y[i], z[i]);
        vbd.emplace_back(x[i + 1], y[i + 1], z[i + 1]);
      }
      cbd.assign(vbd.size(), mrpt::img::TColorf(m_line_R, m_line_G, m_line_B, m_line_A).asTColor());
    }
  }

  // Triangles (surface from sensor origin to scan endpoints)
  {
    std::unique_lock<std::shared_mutex> lck(VisualObjectParams_Triangles::m_trianglesMtx.data);
    auto& tris = VisualObjectParams_Triangles::m_triangles;
    tris.clear();

    if (m_enable_surface && n > 1)
    {
      using P3f = mrpt::math::TPoint3Df;
      const P3f sensorPt(
          d2f(m_scan.sensorPose.x()), d2f(m_scan.sensorPose.y()), d2f(m_scan.sensorPose.z()));

      for (size_t i = 0; i < n - 1; i++)
      {
        tris.emplace_back(sensorPt, P3f(x[i], y[i], z[i]), P3f(x[i + 1], y[i + 1], z[i + 1]));
      }
      for (auto& t : tris)
      {
        t.computeNormals();
        t.setColor(mrpt::img::TColorf(m_plane_R, m_plane_G, m_plane_B, m_plane_A));
      }
    }
  }

  // Points
  {
    std::unique_lock<std::shared_mutex> lck(VisualObjectParams_Points::m_pointsMtx.data);
    auto& vbd = VisualObjectParams_Points::m_vertex_buffer_data;
    auto& cbd = VisualObjectParams_Points::m_color_buffer_data;
    vbd.clear();
    cbd.clear();

    if (m_enable_points && n > 0)
    {
      for (size_t i = 0; i < n; i++)
      {
        vbd.emplace_back(x[i], y[i], z[i]);
      }
      cbd.assign(
          vbd.size(),
          mrpt::img::TColorf(m_points_R, m_points_G, m_points_B, m_points_A).asTColor());
    }
  }
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
