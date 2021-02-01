/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "maps-precomp.h"  // Precomp header
//
#include <mrpt/maps/COccupancyGridMap3D.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>

using namespace mrpt::maps;

double COccupancyGridMap3D::internal_computeObservationLikelihood(
	const mrpt::obs::CObservation& obs,
	const mrpt::poses::CPose3D& takenFrom3D) const
{
	THROW_EXCEPTION("Implement me!");
	return .0;
}

bool COccupancyGridMap3D::internal_canComputeObservationLikelihood(
	const mrpt::obs::CObservation& obs) const
{
	if (auto* o = dynamic_cast<const mrpt::obs::CObservation2DRangeScan*>(&obs);
		o != nullptr)
	{ return true; }

	return false;
}

void COccupancyGridMap3D::TInsertionOptions::loadFromConfigFile(
	const mrpt::config::CConfigFileBase& iniFile, const std::string& section)
{
	MRPT_LOAD_CONFIG_VAR(maxDistanceInsertion, float, iniFile, section);
	MRPT_LOAD_CONFIG_VAR(maxOccupancyUpdateCertainty, float, iniFile, section);
	MRPT_LOAD_CONFIG_VAR(maxFreenessUpdateCertainty, float, iniFile, section);
	MRPT_LOAD_CONFIG_VAR(decimation, int, iniFile, section);
}

void COccupancyGridMap3D::TInsertionOptions::saveToConfigFile(
	mrpt::config::CConfigFileBase& c, const std::string& s) const
{
	MRPT_SAVE_CONFIG_VAR_COMMENT(
		maxDistanceInsertion,
		"Largest distance at which voxels are updated (Default: 15 meters)");
	MRPT_SAVE_CONFIG_VAR_COMMENT(
		maxOccupancyUpdateCertainty,
		"A value in the range [0.5,1] used for updating voxel with a Bayesian "
		"approach (default 0.65)");
	MRPT_SAVE_CONFIG_VAR_COMMENT(
		maxFreenessUpdateCertainty,
		"A value in the range [0.5,1] for updating a free voxel. (default=0 "
		"means use the same than maxOccupancyUpdateCertainty)");
	MRPT_SAVE_CONFIG_VAR_COMMENT(
		decimation,
		"Specify the decimation of the range scan (default=1: take all)");
}
