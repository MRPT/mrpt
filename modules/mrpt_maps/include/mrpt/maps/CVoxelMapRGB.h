/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/img/TColor.h>
#include <mrpt/maps/CVoxelMapOccupancyBase.h>

namespace mrpt::maps
{
/** Voxel contents for CVoxelMapRGB
 */
struct VoxelNodeOccRGB
{
  int8_t occupancy = 0;
  struct TColor
  {
    uint8_t R = 0, G = 0, B = 0;
  } color;
  uint32_t numColObs = 0;

  // ---- API expected by CVoxelMapOccupancyBase ----
  int8_t& occupancyRef() { return occupancy; }
  const int8_t& occupancyRef() const { return occupancy; }
};

/**
 * Log-odds sparse voxel map for cells containing the occupancy *and* an RGB
 * color for each voxel.
 *
 * \ingroup mrpt_maps_grp
 */
class CVoxelMapRGB : public CVoxelMapOccupancyBase<VoxelNodeOccRGB>
{
  // This must be added to any CSerializable derived class:
  DEFINE_SERIALIZABLE(CVoxelMapRGB, mrpt::maps)

 public:
  CVoxelMapRGB(double resolution = 0.05, uint8_t inner_bits = 2, uint8_t leaf_bits = 3) :
      CVoxelMapOccupancyBase(resolution, inner_bits, leaf_bits)
  {
  }
  ~CVoxelMapRGB();

  MAP_DEFINITION_START(CVoxelMapRGB)
  double resolution = 0.10;
  uint8_t inner_bits = 2;
  uint8_t leaf_bits = 3;
  mrpt::maps::TVoxelMap_InsertionOptions insertionOpts;
  mrpt::maps::TVoxelMap_LikelihoodOptions likelihoodOpts;
  MAP_DEFINITION_END(CVoxelMapRGB)

 protected:
  bool internal_insertObservation(
      const mrpt::obs::CObservation& obs,
      const std::optional<const mrpt::poses::CPose3D>& robotPose = std::nullopt) override;

  bool internal_insertObservation_3DScan(
      const mrpt::obs::CObservation3DRangeScan& obs,
      const std::optional<const mrpt::poses::CPose3D>& robotPose = std::nullopt);
  bool internal_insertObservation_default(
      const mrpt::obs::CObservation& obs,
      const std::optional<const mrpt::poses::CPose3D>& robotPose = std::nullopt);

  double internal_computeObservationLikelihood(
      const mrpt::obs::CObservation& obs, const mrpt::poses::CPose3D& takenFrom) const override;
};

}  // namespace mrpt::maps
