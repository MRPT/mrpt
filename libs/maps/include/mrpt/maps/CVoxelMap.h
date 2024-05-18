/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/maps/CVoxelMapOccupancyBase.h>

namespace mrpt::obs
{
class CObservationPointCloud;
}

namespace mrpt::maps
{
/** Voxel contents for CVoxelMap
 */
struct VoxelNodeOccupancy
{
  int8_t occupancy = 0;

  // ---- API expected by CVoxelMapOccupancyBase ----
  int8_t& occupancyRef() { return occupancy; }
  const int8_t& occupancyRef() const { return occupancy; }
};

/**
 * Log-odds sparse voxel map for cells containing only the *occupancy* of each
 * voxel.
 *
 * \ingroup mrpt_maps_grp
 */
class CVoxelMap : public CVoxelMapOccupancyBase<VoxelNodeOccupancy>
{
  // This must be added to any CSerializable derived class:
  DEFINE_SERIALIZABLE(CVoxelMap, mrpt::maps)

 public:
  CVoxelMap(double resolution = 0.05, uint8_t inner_bits = 2, uint8_t leaf_bits = 3) :
      CVoxelMapOccupancyBase(resolution, inner_bits, leaf_bits)
  {
  }
  ~CVoxelMap();

  MAP_DEFINITION_START(CVoxelMap)
  double resolution = 0.10;
  uint8_t inner_bits = 2;
  uint8_t leaf_bits = 3;
  mrpt::maps::TVoxelMap_InsertionOptions insertionOpts;
  mrpt::maps::TVoxelMap_LikelihoodOptions likelihoodOpts;
  MAP_DEFINITION_END(CVoxelMap)

 protected:
  bool internal_insertObservation(
      const mrpt::obs::CObservation& obs,
      const std::optional<const mrpt::poses::CPose3D>& robotPose = std::nullopt) override;
  bool internal_insertObservation_Pts(
      const mrpt::obs::CObservationPointCloud& obs,
      const std::optional<const mrpt::poses::CPose3D>& robotPose = std::nullopt);
  double internal_computeObservationLikelihood(
      const mrpt::obs::CObservation& obs, const mrpt::poses::CPose3D& takenFrom) const override;
};

}  // namespace mrpt::maps
