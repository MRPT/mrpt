/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "maps-precomp.h"  // Precomp header
//
#include <mrpt/maps/CColouredPointsMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/CVoxelMapRGB.h>
#include <mrpt/obs/CObservation3DRangeScan.h>

#include <mrpt/maps/bonxai/serialization.hpp>

using namespace mrpt::maps;
using namespace std::string_literals;  // "..."s

//  =========== Begin of Map definition ============
MAP_DEFINITION_REGISTER(
	"mrpt::maps::CVoxelMapRGB,voxelMapRGB", mrpt::maps::CVoxelMapRGB)

CVoxelMapRGB::TMapDefinition::TMapDefinition() = default;
void CVoxelMapRGB::TMapDefinition::loadFromConfigFile_map_specific(
	const mrpt::config::CConfigFileBase& source,
	const std::string& sectionNamePrefix)
{
	// [<sectionNamePrefix>+"_creationOpts"]
	const std::string sSectCreation = sectionNamePrefix + "_creationOpts"s;
	MRPT_LOAD_CONFIG_VAR(resolution, double, source, sSectCreation);

	insertionOpts.loadFromConfigFile(
		source, sectionNamePrefix + "_insertOpts"s);
	likelihoodOpts.loadFromConfigFile(
		source, sectionNamePrefix + "_likelihoodOpts"s);
}

void CVoxelMapRGB::TMapDefinition::dumpToTextStream_map_specific(
	std::ostream& out) const
{
	LOADABLEOPTS_DUMP_VAR(resolution, double);

	this->insertionOpts.dumpToTextStream(out);
	this->likelihoodOpts.dumpToTextStream(out);
}

mrpt::maps::CMetricMap* CVoxelMapRGB::internal_CreateFromMapDefinition(
	const mrpt::maps::TMetricMapInitializer& _def)
{
	const CVoxelMapRGB::TMapDefinition& def =
		*dynamic_cast<const CVoxelMapRGB::TMapDefinition*>(&_def);
	auto* obj = new CVoxelMapRGB(def.resolution);
	obj->insertionOptions = def.insertionOpts;
	obj->likelihoodOptions = def.likelihoodOpts;
	return obj;
}
//  =========== End of Map definition Block =========

IMPLEMENTS_SERIALIZABLE(CVoxelMapRGB, CMetricMap, mrpt::maps)

/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
CVoxelMapRGB::~CVoxelMapRGB() = default;

uint8_t CVoxelMapRGB::serializeGetVersion() const { return 0; }
void CVoxelMapRGB::serializeTo(mrpt::serialization::CArchive& out) const
{
	insertionOptions.writeToStream(out);
	likelihoodOptions.writeToStream(out);
	renderingOptions.writeToStream(out);  // Added in v1
	out << genericMapParams;

	// grid data:
	std::stringstream ss;
	Bonxai::Serialize(ss, grid());
	out << ss.str();
}

void CVoxelMapRGB::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		{
			insertionOptions.readFromStream(in);
			likelihoodOptions.readFromStream(in);
			renderingOptions.readFromStream(in);
			in >> genericMapParams;

			this->clear();

			// grid data:
			std::string msg;
			in >> msg;
			std::istringstream ifile(msg, std::ios::binary);

			char header[256];
			ifile.getline(header, 256);
			Bonxai::HeaderInfo info = Bonxai::GetHeaderInfo(header);

			m_impl = std::make_unique<Impl>(
				Bonxai::Deserialize<voxel_node_t>(ifile, info));
		}
		break;
		default: MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
}

bool CVoxelMapRGB::internal_insertObservation_3DScan(
	const mrpt::obs::CObservation3DRangeScan& obs,
	const std::optional<const mrpt::poses::CPose3D>& robotPose)
{
	mrpt::maps::CColouredPointsMap colPts;

	mrpt::obs::T3DPointsProjectionParams pp;
	pp.takeIntoAccountSensorPoseOnRobot = true;
	pp.robotPoseInTheWorld = robotPose;

	// remove const (keep walking...)
	auto& oobs = const_cast<mrpt::obs::CObservation3DRangeScan&>(obs);
	oobs.unprojectInto(colPts, pp);

	if (colPts.empty()) return false;

	// Insert occupancy info + RGB color:
	invalidateOccupiedCache();

	const occupancy_value_t logodd_observation_occupied =
		std::max<occupancy_value_t>(1, p2l(insertionOptions.prob_hit));
	const occupancy_value_t logodd_thres_occupied =
		p2l(1.0 - insertionOptions.clamp_max);

	const auto& xs = colPts.getPointsBufferRef_x();
	const auto& ys = colPts.getPointsBufferRef_y();
	const auto& zs = colPts.getPointsBufferRef_z();

	const auto maxSqrDist = mrpt::square(insertionOptions.max_range);
	mrpt::math::TPoint3D sensorPt;
	mrpt::poses::CPose3D localSensorPose;
	obs.getSensorPose(localSensorPose);
	if (robotPose)
	{
		// compose:
		sensorPt = (*robotPose + localSensorPose).translation();
	}
	else
	{
		sensorPt = localSensorPose.translation();
	}

	for (size_t i = 0; i < xs.size(); i += insertionOptions.decimation)
	{
		if (insertionOptions.max_range > 0 &&
			(mrpt::math::TPoint3D(xs[i], ys[i], zs[i]) - sensorPt).sqrNorm() >
				maxSqrDist)
			continue;  // skip

		voxel_node_t* cell = base_t::m_impl->accessor.value(
			Bonxai::PosToCoord(
				{xs[i], ys[i], zs[i]}, base_t::m_impl->grid.inv_resolution),
			true /*create*/);
		if (!cell) continue;  // should never happen?

		// Update occupancy:
		updateCell_fast_occupied(
			cell, logodd_observation_occupied, logodd_thres_occupied);

		// and merge color:
		mrpt::img::TColorf colF;
		colPts.getPointColor(i, colF.R, colF.G, colF.B);
#if 1  // fuse colors:
		mrpt::img::TColorf oldCol(
			mrpt::img::TColor(cell->color.R, cell->color.G, cell->color.B));

		mrpt::img::TColorf newF;
		const float N_1 = 1.0f / (cell->numColObs + 1);

		newF.R = N_1 * (oldCol.R * cell->numColObs + colF.R);
		newF.G = N_1 * (oldCol.G * cell->numColObs + colF.G);
		newF.B = N_1 * (oldCol.B * cell->numColObs + colF.B);

		cell->numColObs++;
		const auto nCol = newF.asTColor();
		cell->color.R = nCol.R;
		cell->color.G = nCol.G;
		cell->color.B = nCol.B;
#else
		// just copy latest color:
		cell->color = colF.asTColor();
#endif
	}

	return true;
}

bool CVoxelMapRGB::internal_insertObservation(
	const mrpt::obs::CObservation& obs,
	const std::optional<const mrpt::poses::CPose3D>& robotPose)
{
	// Auxiliary 3D point cloud:
	if (auto obs3Dscan =
			dynamic_cast<const mrpt::obs::CObservation3DRangeScan*>(&obs);
		obs3Dscan)
	{
		return internal_insertObservation_3DScan(*obs3Dscan, robotPose);
	}
	else
	{
		return internal_insertObservation_default(obs, robotPose);
	}
}

bool CVoxelMapRGB::internal_insertObservation_default(
	const mrpt::obs::CObservation& obs,
	const std::optional<const mrpt::poses::CPose3D>& robotPose)
{
	// build aux 3D pointcloud:
	mrpt::maps::CSimplePointsMap pts;
	pts.insertObservation(obs, robotPose);

	if (pts.empty()) return false;

	mrpt::math::TPoint3D sensorPt;
	mrpt::poses::CPose3D localSensorPose;
	obs.getSensorPose(localSensorPose);
	if (robotPose)
	{
		// compose:
		sensorPt = (*robotPose + localSensorPose).translation();
	}
	else
	{
		sensorPt = localSensorPose.translation();
	}

	// Insert rays:
	if (insertionOptions.ray_trace_free_space)
		insertPointCloudAsRays(pts, sensorPt);
	else
		insertPointCloudAsEndPoints(pts, sensorPt);
	return true;
}

double CVoxelMapRGB::internal_computeObservationLikelihood(
	const mrpt::obs::CObservation& obs,
	const mrpt::poses::CPose3D& takenFrom) const
{
	THROW_EXCEPTION("TODO");
	return 0;
}
