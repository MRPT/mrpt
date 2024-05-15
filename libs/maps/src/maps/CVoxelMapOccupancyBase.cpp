/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "maps-precomp.h"  // Precomp header
//
#include <mrpt/maps/CVoxelMapOccupancyBase.h>

using namespace mrpt::maps;

void TVoxelMap_InsertionOptions::loadFromConfigFile(
    const mrpt::config::CConfigFileBase& c, const std::string& s)
{
  MRPT_LOAD_CONFIG_VAR(max_range, double, c, s);
  MRPT_LOAD_CONFIG_VAR(prob_miss, double, c, s);
  MRPT_LOAD_CONFIG_VAR(prob_hit, double, c, s);
  MRPT_LOAD_CONFIG_VAR(clamp_min, double, c, s);
  MRPT_LOAD_CONFIG_VAR(clamp_max, double, c, s);
  MRPT_LOAD_CONFIG_VAR(ray_trace_free_space, bool, c, s);
  MRPT_LOAD_CONFIG_VAR(decimation, uint64_t, c, s);
  MRPT_LOAD_CONFIG_VAR(remove_voxels_farther_than, double, c, s);
}
void TVoxelMap_InsertionOptions::saveToConfigFile(
    mrpt::config::CConfigFileBase& c, const std::string& s) const
{
  MRPT_SAVE_CONFIG_VAR(max_range, c, s);
  MRPT_SAVE_CONFIG_VAR(prob_miss, c, s);
  MRPT_SAVE_CONFIG_VAR(prob_hit, c, s);
  MRPT_SAVE_CONFIG_VAR(clamp_min, c, s);
  MRPT_SAVE_CONFIG_VAR(clamp_max, c, s);
  MRPT_SAVE_CONFIG_VAR(ray_trace_free_space, c, s);
  MRPT_SAVE_CONFIG_VAR(decimation, c, s);
  MRPT_SAVE_CONFIG_VAR(remove_voxels_farther_than, c, s);
}

void TVoxelMap_InsertionOptions::writeToStream(mrpt::serialization::CArchive& out) const
{
  const uint8_t version = 1;
  out << version;

  out << max_range << prob_miss << prob_hit << clamp_min << clamp_max;
  out << ray_trace_free_space << decimation;
  out << remove_voxels_farther_than;  // v1
}

void TVoxelMap_InsertionOptions::readFromStream(mrpt::serialization::CArchive& in)
{
  const uint8_t version = in.ReadAs<uint8_t>();
  switch (version)
  {
    case 0:
    case 1:
      in >> max_range >> prob_miss >> prob_hit >> clamp_min >> clamp_max;
      in >> ray_trace_free_space >> decimation;
      if (version >= 1)
      {
        in >> remove_voxels_farther_than;
      }
      break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  }
}

void TVoxelMap_RenderingOptions::writeToStream(mrpt::serialization::CArchive& out) const
{
  const uint8_t version = 0;
  out << version;

  out << generateOccupiedVoxels << visibleOccupiedVoxels;
  out << generateFreeVoxels << visibleFreeVoxels;
  out << occupiedThreshold << freeThreshold;
}

void TVoxelMap_RenderingOptions::readFromStream(mrpt::serialization::CArchive& in)
{
  const uint8_t version = in.ReadAs<uint8_t>();
  switch (version)
  {
    case 0:
      in >> generateOccupiedVoxels >> visibleOccupiedVoxels;
      in >> generateFreeVoxels >> visibleFreeVoxels;
      in >> occupiedThreshold >> freeThreshold;
      break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  }
}

void TVoxelMap_LikelihoodOptions::loadFromConfigFile(
    const mrpt::config::CConfigFileBase& c, const std::string& s)
{
  MRPT_LOAD_CONFIG_VAR(decimate_up_to, int, c, s);
  MRPT_LOAD_CONFIG_VAR(occupiedThreshold, double, c, s);
}
void TVoxelMap_LikelihoodOptions::saveToConfigFile(
    mrpt::config::CConfigFileBase& c, const std::string& s) const
{
  MRPT_SAVE_CONFIG_VAR(decimate_up_to, c, s);
  MRPT_SAVE_CONFIG_VAR(occupiedThreshold, c, s);
}

void TVoxelMap_LikelihoodOptions::writeToStream(mrpt::serialization::CArchive& out) const
{
  out << decimate_up_to << occupiedThreshold;
}

void TVoxelMap_LikelihoodOptions::readFromStream(mrpt::serialization::CArchive& in)
{
  in >> decimate_up_to >> occupiedThreshold;
}
