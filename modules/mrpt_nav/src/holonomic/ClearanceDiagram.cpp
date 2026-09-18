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

#include <mrpt/core/round.h>
#include <mrpt/nav/holonomic/ClearanceDiagram.h>
#include <mrpt/nav/tpspace/CParameterizedTrajectoryGenerator.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/stl_serialization.h>
#include <mrpt/viz/CMesh.h>

#include <limits>

using namespace mrpt::nav;

ClearanceDiagram::ClearanceDiagram()

    = default;

void ClearanceDiagram::renderAs3DObject(
    mrpt::viz::CMesh& mesh,
    double min_x,
    double max_x,
    double min_y,
    double max_y,
    double cell_res,
    ClearanceQuery mode) const
{
  ASSERT_(cell_res > 0.0);
  ASSERT_(max_x > min_x);
  ASSERT_(max_y > min_y);

  mesh.setXBounds(static_cast<float>(min_x), static_cast<float>(max_x));
  mesh.setYBounds(static_cast<float>(min_y), static_cast<float>(max_y));
  const int nX = static_cast<int>(::ceil((max_x - min_x) / cell_res));
  const int nY = static_cast<int>(::ceil((max_y - min_y) / cell_res));
  const double dx = (max_x - min_x) / nX;
  const double dy = (max_y - min_y) / nY;

  mrpt::math::CMatrixFloat Z(nX, nY);

  if (m_raw_clearances.empty()) return;  // Nothing to do: empty structure!

  for (int iX = 0; iX < nX; iX++)
  {
    const double x = min_x + dx * (0.5 + iX);
    for (int iY = 0; iY < nY; iY++)
    {
      const double y = min_y + dy * (0.5 + iY);

      double clear_val = .0;
      if (x != 0 || y != 0)
      {
        const double alpha = ::atan2(y, x);
        const uint16_t actual_k = CParameterizedTrajectoryGenerator::Alpha2index(
            alpha, static_cast<unsigned int>(m_actual_num_paths));
        const double dist = std::hypot(x, y);
        clear_val = this->getClearance(actual_k, dist, mode);
      }
      Z(iX, iY) = static_cast<float>(clear_val);
    }
  }

  mesh.setZ(Z);
  mesh.enableColorFromZ(true);
  mesh.enableTransparency(true);
  mesh.setColorA_u8(0x50);
  mesh.enableWireFrame(false);
}

void mrpt::nav::ClearanceDiagram::readFromStream(mrpt::serialization::CArchive& in)
{
  uint8_t version;
  in >> version;
  switch (version)
  {
    case 0:
      uint32_t decim_num;
      in.ReadAsAndCastTo<uint32_t, size_t>(m_actual_num_paths);
      in >> decim_num;
      this->resize(m_actual_num_paths, decim_num);
      in >> m_raw_clearances;
      break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };
}

void mrpt::nav::ClearanceDiagram::writeToStream(mrpt::serialization::CArchive& out) const
{
  const uint8_t version = 0;
  out << version;

  out << uint32_t(m_actual_num_paths) << uint32_t(m_raw_clearances.size());
  out << m_raw_clearances;
}

ClearanceDiagram::dist2clearance_t& ClearanceDiagram::get_path_clearance(size_t actual_k)
{
  return m_raw_clearances[real_k_to_decimated_k(actual_k)];
}

const ClearanceDiagram::dist2clearance_t& ClearanceDiagram::get_path_clearance(
    size_t actual_k) const
{
  return m_raw_clearances[real_k_to_decimated_k(actual_k)];
}

size_t mrpt::nav::ClearanceDiagram::real_k_to_decimated_k(size_t k) const
{
  ASSERT_(m_actual_num_paths > 0 && !m_raw_clearances.empty());
  const size_t ret = mrpt::round(static_cast<double>(k) * m_k_a2d);
  ASSERT_(ret < m_raw_clearances.size());
  return ret;
}

size_t mrpt::nav::ClearanceDiagram::decimated_k_to_real_k(size_t k) const
{
  ASSERT_(m_actual_num_paths > 0 && !m_raw_clearances.empty());
  const size_t ret = mrpt::round(static_cast<double>(k) * m_k_d2a);
  ASSERT_(ret < m_actual_num_paths);
  return ret;
}

double ClearanceDiagram::getClearance(uint16_t actual_k, double dist, ClearanceQuery mode) const
{
  if (this->empty())  // If we are not using clearance values, just return a
    // fixed value:
    return 0.0;

  ASSERT_LT_(actual_k, m_actual_num_paths);

  const auto& rc_k = m_raw_clearances[real_k_to_decimated_k(actual_k)];
  if (rc_k.empty()) return 0.0;

  double sum = 0;
  double last = 0;
  size_t count = 0;
  for (const auto& [sample_dist, sample_clearance] : rc_k)
  {
    sum += sample_clearance;
    last = sample_clearance;
    count++;
    if (sample_dist > dist) break;  // target dist reached.
  }

  return mode == ClearanceQuery::MeanUpToDistance ? sum / static_cast<double>(count) : last;
}

void ClearanceDiagram::clear()
{
  m_actual_num_paths = 0;
  m_raw_clearances.clear();
  m_k_a2d = m_k_d2a = .0;
}

void mrpt::nav::ClearanceDiagram::resize(size_t actual_num_paths, size_t decimated_num_paths)
{
  if (decimated_num_paths == 0)
  {
    this->clear();
    return;
  }
  ASSERT_GE_(actual_num_paths, decimated_num_paths);

  m_actual_num_paths = actual_num_paths;
  m_raw_clearances.resize(decimated_num_paths);

  if (m_actual_num_paths > 1 && m_raw_clearances.size() > 1)
  {
    m_k_d2a = static_cast<double>(m_actual_num_paths - 1) /
              static_cast<double>(m_raw_clearances.size() - 1);
    m_k_a2d = static_cast<double>(m_raw_clearances.size() - 1) /
              static_cast<double>(m_actual_num_paths - 1);
  }
  else
  {
    // Degenerate case: one single (decimated) path, everything maps to index 0.
    m_k_d2a = m_k_a2d = .0;
  }
}
