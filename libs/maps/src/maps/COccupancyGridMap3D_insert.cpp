/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "maps-precomp.h"  // Precomp header

#include <mrpt/maps/COccupancyGridMap3D.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/poses/CPose3D.h>

using namespace mrpt::maps;

// bits to left-shift for fixed-point arithmetic simulation in raytracing.
static constexpr unsigned FRBITS = 9;

bool COccupancyGridMap3D::internal_insertObservation(
	const mrpt::obs::CObservation& obs, const mrpt::poses::CPose3D* robPose)
{
	MRPT_START

	const mrpt::poses::CPose3D robotPose3D =
		(robPose != nullptr) ? *robPose : mrpt::poses::CPose3D();

	if (auto* o = dynamic_cast<const mrpt::obs::CObservation2DRangeScan*>(&obs);
		o != nullptr)
	{
		this->internal_insertObservationScan2D(*o, robotPose3D);
		return true;
	}
	if (auto* o = dynamic_cast<const mrpt::obs::CObservation3DRangeScan*>(&obs);
		o != nullptr)
	{
		this->internal_insertObservationScan3D(*o, robotPose3D);
		return true;
	}

	return false;

	MRPT_END
}

void COccupancyGridMap3D::internal_insertObservationScan2D(
	const mrpt::obs::CObservation2DRangeScan& o,
	const mrpt::poses::CPose3D& robotPose)
{
	MRPT_START

	// Convert to point cloud:
	mrpt::maps::CSimplePointsMap pts;
	pts.insertionOptions.also_interpolate = false;
	pts.insertionOptions.addToExistingPointsMap = true;
	pts.insertionOptions.disableDeletion = true;
	pts.insertionOptions.fuseWithExisting = false;
	pts.insertionOptions.insertInvalidPoints = false;
	pts.insertionOptions.minDistBetweenLaserPoints = .0;  // insert all

	pts.loadFromRangeScan(o, &robotPose);

	const auto sensorPose3D = robotPose + o.sensorPose;
	const auto sensorPt = mrpt::math::TPoint3D(sensorPose3D.asTPose());
	insertPointCloud(sensorPt, pts);

	MRPT_END
}

void COccupancyGridMap3D::insertPointCloud(
	const mrpt::math::TPoint3D& sensorPt, const mrpt::maps::CPointsMap& pts,
	const float maxValidRange)
{
	MRPT_START

	const auto& xs = pts.getPointsBufferRef_x();
	const auto& ys = pts.getPointsBufferRef_y();
	const auto& zs = pts.getPointsBufferRef_z();

	// Process points one by one as rays:
	for (std::size_t idx = 0; idx < xs.size();
		 idx += insertionOptions.decimation)
	{
		insertRay(sensorPt, mrpt::math::TPoint3D(xs[idx], ys[idx], zs[idx]));
	}

	MRPT_END
}

void COccupancyGridMap3D::internal_insertObservationScan3D(
	const mrpt::obs::CObservation3DRangeScan& o,
	const mrpt::poses::CPose3D& robotPose)
{
	MRPT_START

	// Depth -> 3D points:
	mrpt::maps::CSimplePointsMap pts;
	mrpt::obs::T3DPointsProjectionParams pp;
	pp.takeIntoAccountSensorPoseOnRobot = false;  // done below
	pp.decimation = insertionOptions.decimation_3d_range;

	const_cast<mrpt::obs::CObservation3DRangeScan&>(o).unprojectInto(pts, pp);

	const auto sensorPose3D = robotPose + o.sensorPose;
	// Shift everything to its proper pose in the global frame:
	pts.changeCoordinatesReference(sensorPose3D);

	const auto sensorPt = mrpt::math::TPoint3D(sensorPose3D.asTPose());
	insertPointCloud(sensorPt, pts, o.maxRange);

	MRPT_END
}

void COccupancyGridMap3D::insertRay(
	const mrpt::math::TPoint3D& sensor, const mrpt::math::TPoint3D& end,
	bool endIsOccupied)
{
	MRPT_START

	// m_likelihoodCacheOutDated = true;

	// the occupied and free probabilities:
	const float maxCertainty = insertionOptions.maxOccupancyUpdateCertainty;
	float maxFreeCertainty = insertionOptions.maxFreenessUpdateCertainty;
	if (maxFreeCertainty == .0f) maxFreeCertainty = maxCertainty;

	const voxelType logodd_observation_free =
		std::max<voxelType>(1, p2l(maxFreeCertainty));
	const voxelType logodd_observation_occupied =
		3 * std::max<voxelType>(1, p2l(maxCertainty));

	// saturation limits:
	const voxelType logodd_thres_occupied =
		CLogOddsGridMap3D<voxelType>::CELLTYPE_MIN +
		logodd_observation_occupied;
	const voxelType logodd_thres_free =
		CLogOddsGridMap3D<voxelType>::CELLTYPE_MAX - logodd_observation_free;

	// Start: (in cell index units)
	int cx = m_grid.x2idx(sensor.x);
	int cy = m_grid.y2idx(sensor.y);
	int cz = m_grid.z2idx(sensor.z);

	// End: (in cell index units)
	int trg_cx = m_grid.x2idx(end.x);
	int trg_cy = m_grid.y2idx(end.y);
	int trg_cz = m_grid.z2idx(end.z);

	// Skip if totally out of bounds:
	if (m_grid.isOutOfBounds(cx, cy, cz)) return;

	// Use "fractional integers" to approximate float operations
	//  during the ray tracing:
	const int Acx = trg_cx - cx;
	const int Acy = trg_cy - cy;
	const int Acz = trg_cz - cz;

	const int Acx_ = std::abs(Acx);
	const int Acy_ = std::abs(Acy);
	const int Acz_ = std::abs(Acz);

	const int nStepsRay = mrpt::max3(Acx_, Acy_, Acz_);
	if (!nStepsRay) return;  // May be...

	const float N_1 = 1.0f / nStepsRay;

	// Increments at each raytracing step:
	const int frAcx = (Acx < 0 ? -1 : +1) * round((Acx_ << FRBITS) * N_1);
	const int frAcy = (Acy < 0 ? -1 : +1) * round((Acy_ << FRBITS) * N_1);
	const int frAcz = (Acz < 0 ? -1 : +1) * round((Acz_ << FRBITS) * N_1);

	// fractional integers for the running raytracing point:
	int frCX = cx << FRBITS;
	int frCY = cy << FRBITS;
	int frCZ = cz << FRBITS;

	for (int nStep = 0; nStep < nStepsRay; nStep++)
	{
		updateCell_fast_free(
			cx, cy, cz, logodd_observation_free, logodd_thres_free);

		frCX += frAcx;
		frCY += frAcy;
		frCZ += frAcz;

		cx = frCX >> FRBITS;
		cy = frCY >> FRBITS;
		cz = frCZ >> FRBITS;

		// Already out of bounds?
		if (m_grid.isOutOfBounds(cx, cy, cz)) break;
	}

	// And finally, the occupied cell at the end:
	updateCell_fast_occupied(
		trg_cx, trg_cy, trg_cz, logodd_observation_occupied,
		logodd_thres_occupied);

	MRPT_END
}

void COccupancyGridMap3D::OnPostSuccesfulInsertObs(
	const mrpt::obs::CObservation&)
{
	m_is_empty = false;
}

void COccupancyGridMap3D::TLikelihoodOptions::loadFromConfigFile(
	const mrpt::config::CConfigFileBase& iniFile, const std::string& section)
{
	likelihoodMethod =
		iniFile.read_enum(section, "likelihoodMethod", likelihoodMethod);

	MRPT_LOAD_CONFIG_VAR(LF_stdHit, float, iniFile, section);
	MRPT_LOAD_CONFIG_VAR(LF_zHit, float, iniFile, section);
	MRPT_LOAD_CONFIG_VAR(LF_zRandom, float, iniFile, section);
	MRPT_LOAD_CONFIG_VAR(LF_maxRange, int, iniFile, section);
	MRPT_LOAD_CONFIG_VAR(LF_decimation, int, iniFile, section);
	MRPT_LOAD_CONFIG_VAR(LF_maxCorrsDistance, int, iniFile, section);
	MRPT_LOAD_CONFIG_VAR(LF_useSquareDist, int, iniFile, section);

	MRPT_LOAD_CONFIG_VAR(rayTracing_stdHit, int, iniFile, section);
	MRPT_LOAD_CONFIG_VAR(rayTracing_decimation, int, iniFile, section);
}

void COccupancyGridMap3D::TLikelihoodOptions::saveToConfigFile(
	mrpt::config::CConfigFileBase& c, const std::string& s) const
{
	MRPT_SAVE_CONFIG_VAR_COMMENT(LF_stdHit, "");
	MRPT_SAVE_CONFIG_VAR_COMMENT(LF_zHit, "");
	MRPT_SAVE_CONFIG_VAR_COMMENT(LF_zRandom, "");
	MRPT_SAVE_CONFIG_VAR_COMMENT(LF_maxRange, "");
	MRPT_SAVE_CONFIG_VAR_COMMENT(LF_decimation, "");
	MRPT_SAVE_CONFIG_VAR_COMMENT(LF_maxCorrsDistance, "");
	MRPT_SAVE_CONFIG_VAR_COMMENT(LF_useSquareDist, "");

	MRPT_SAVE_CONFIG_VAR_COMMENT(rayTracing_stdHit, "");
	MRPT_SAVE_CONFIG_VAR_COMMENT(rayTracing_decimation, "");
}
