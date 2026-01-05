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
#include <mrpt/serialization/stl_serialization.h>
#include <mrpt/viz/COctoMapVoxels.h>

using namespace mrpt;
using namespace mrpt::viz;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(COctoMapVoxels, CVisualObject, mrpt::viz)

/** Ctor */
COctoMapVoxels::COctoMapVoxels() : m_grid_color(0xE0, 0xE0, 0xE0, 0x90) {}
/** Clears everything */
void COctoMapVoxels::clear()
{
  m_voxel_sets.clear();
  m_grid_cubes.clear();

  CVisualObject::notifyChange();
}

void COctoMapVoxels::setBoundingBox(
    const mrpt::math::TPoint3D& bb_min, const mrpt::math::TPoint3D& bb_max)
{
  m_bb_min = bb_min;
  m_bb_max = bb_max;
}

DECLARE_CUSTOM_TTYPENAME(COctoMapVoxels::TInfoPerVoxelSet)
DECLARE_CUSTOM_TTYPENAME(COctoMapVoxels::TGridCube)
DECLARE_CUSTOM_TTYPENAME(COctoMapVoxels::TVoxel)

namespace mrpt::viz
{
using mrpt::serialization::CArchive;
CArchive& operator<<(CArchive& out, const COctoMapVoxels::TInfoPerVoxelSet& a)
{
  out << a.visible << a.voxels;
  return out;
}
CArchive& operator>>(CArchive& in, COctoMapVoxels::TInfoPerVoxelSet& a)
{
  in >> a.visible >> a.voxels;
  return in;
}

CArchive& operator<<(CArchive& out, const COctoMapVoxels::TGridCube& a)
{
  out << a.min << a.max;
  return out;
}
CArchive& operator>>(CArchive& in, COctoMapVoxels::TGridCube& a)
{
  in >> a.min >> a.max;
  return in;
}

CArchive& operator<<(CArchive& out, const COctoMapVoxels::TVoxel& a)
{
  out << a.coords << a.side_length << a.color;
  return out;
}
CArchive& operator>>(CArchive& in, COctoMapVoxels::TVoxel& a)
{
  in >> a.coords >> a.side_length >> a.color;
  return in;
}
}  // end of namespace mrpt::viz

uint8_t COctoMapVoxels::serializeGetVersion() const { return 4; }
void COctoMapVoxels::serializeTo(CArchive& out) const
{
  writeToStreamRender(out);

  out << m_voxel_sets << m_grid_cubes << m_bb_min << m_bb_max << m_enable_lighting
      << m_showVoxelsAsPoints << m_showVoxelsAsPointsSize << m_show_grids << m_grid_width
      << m_grid_color << m_enable_cube_transparency     // added in v1
      << uint32_t(m_visual_mode);                       // added in v2
  VisualObjectParams_Triangles::params_serialize(out);  // v3
  out.WriteAs<uint8_t>(m_color_map);                    // v4
}

void COctoMapVoxels::serializeFrom(CArchive& in, uint8_t version)
{
  switch (version)
  {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    {
      readFromStreamRender(in);

      in >> m_voxel_sets >> m_grid_cubes >> m_bb_min >> m_bb_max >> m_enable_lighting >>
          m_showVoxelsAsPoints >> m_showVoxelsAsPointsSize >> m_show_grids >> m_grid_width >>
          m_grid_color;

      if (version >= 1)
        in >> m_enable_cube_transparency;
      else
        m_enable_cube_transparency = false;

      if (version >= 2)
      {
        uint32_t i;
        in >> i;
        m_visual_mode = static_cast<COctoMapVoxels::visualization_mode_t>(i);
      }
      else
        m_visual_mode = COctoMapVoxels::COLOR_FROM_OCCUPANCY;

      if (version >= 3) VisualObjectParams_Triangles::params_deserialize(in);

      if (version >= 4) m_color_map = static_cast<mrpt::img::TColormap>(in.ReadAs<uint8_t>());
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };

  CVisualObject::notifyChange();
}

auto COctoMapVoxels::internalBoundingBoxLocal() const -> mrpt::math::TBoundingBoxf
{
  return mrpt::math::TBoundingBoxf::FromUnsortedPoints(m_bb_min, m_bb_max);
}

bool sort_voxels_z(const COctoMapVoxels::TVoxel& a, const COctoMapVoxels::TVoxel& b)
{
  return a.coords.z < b.coords.z;
}

void COctoMapVoxels::sort_voxels_by_z()
{
  for (auto& m_voxel_set : m_voxel_sets)
  {
    std::sort(m_voxel_set.voxels.begin(), m_voxel_set.voxels.end(), &sort_voxels_z);
  }
}
