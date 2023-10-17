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
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/CVoxelMap.h>

using namespace mrpt::maps;
using namespace std::string_literals;  // "..."s

//  =========== Begin of Map definition ============
MAP_DEFINITION_REGISTER("mrpt::maps::CVoxelMap,voxelMap", mrpt::maps::CVoxelMap)

CVoxelMap::TMapDefinition::TMapDefinition() = default;
void CVoxelMap::TMapDefinition::loadFromConfigFile_map_specific(
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

void CVoxelMap::TMapDefinition::dumpToTextStream_map_specific(
	std::ostream& out) const
{
	LOADABLEOPTS_DUMP_VAR(resolution, double);

	this->insertionOpts.dumpToTextStream(out);
	this->likelihoodOpts.dumpToTextStream(out);
}

mrpt::maps::CMetricMap* CVoxelMap::internal_CreateFromMapDefinition(
	const mrpt::maps::TMetricMapInitializer& _def)
{
	const CVoxelMap::TMapDefinition& def =
		*dynamic_cast<const CVoxelMap::TMapDefinition*>(&_def);
	auto* obj = new CVoxelMap(def.resolution);
	obj->insertionOptions = def.insertionOpts;
	obj->likelihoodOptions = def.likelihoodOpts;
	return obj;
}
//  =========== End of Map definition Block =========

IMPLEMENTS_SERIALIZABLE(CVoxelMap, CMetricMap, mrpt::maps)

// Static lookup tables for log-odds
static CLogOddsGridMapLUT<CVoxelMap::voxel_node_t> logodd_lut;

CLogOddsGridMapLUT<CVoxelMap::voxel_node_t>& CVoxelMap::get_logodd_lut()
{
	return logodd_lut;
}

/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
CVoxelMap::~CVoxelMap() = default;

uint8_t CVoxelMap::serializeGetVersion() const { return 0; }
void CVoxelMap::serializeTo(mrpt::serialization::CArchive& out) const
{
	insertionOptions.writeToStream(out);
	likelihoodOptions.writeToStream(out);
	renderingOptions.writeToStream(out);  // Added in v1
	out << genericMapParams;

	THROW_EXCEPTION("TODO");
	// const_cast<octomap::OcTree*>(&m_impl->m_octomap)->writeBinary(ss);
}

void CVoxelMap::serializeFrom(
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

			THROW_EXCEPTION("TODO");
			// m_impl->m_octomap.readBinary(ss);
		}
		break;
		default: MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
}

bool CVoxelMap::internal_insertObservation(
	const mrpt::obs::CObservation& obs,
	const std::optional<const mrpt::poses::CPose3D>& robotPose)
{
	// Auxiliary 3D point cloud:
	MRPT_TODO("Handle special cases and avoid duplicating pointcloud?");

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
		insertPointCloudAsEndPoints(pts);
	return true;
}

double CVoxelMap::internal_computeObservationLikelihood(
	const mrpt::obs::CObservation& obs,
	const mrpt::poses::CPose3D& takenFrom) const
{
	THROW_EXCEPTION("TODO");
	return 0;
}

bool CVoxelMap::isEmpty() const
{
	THROW_EXCEPTION("TODO");
	return false;
}

void CVoxelMap::getAsOctoMapVoxels(mrpt::opengl::COctoMapVoxels& gl_obj) const
{
	using mrpt::opengl::COctoMapVoxels;
	using mrpt::opengl::VOXEL_SET_FREESPACE;
	using mrpt::opengl::VOXEL_SET_OCCUPIED;

	const mrpt::img::TColorf general_color = gl_obj.getColor();
	const mrpt::img::TColor general_color_u = general_color.asTColor();

	gl_obj.clear();
	gl_obj.resizeVoxelSets(2);	// 2 sets of voxels: occupied & free

	gl_obj.showVoxels(
		mrpt::opengl::VOXEL_SET_OCCUPIED,
		renderingOptions.visibleOccupiedVoxels);
	gl_obj.showVoxels(
		mrpt::opengl::VOXEL_SET_FREESPACE, renderingOptions.visibleFreeVoxels);

	const size_t nLeafs = m_impl->grid.activeCellsCount();
	gl_obj.reserveVoxels(VOXEL_SET_OCCUPIED, nLeafs);

	mrpt::math::TBoundingBox bbox =
		mrpt::math::TBoundingBox::PlusMinusInfinity();

	// forEachCell() has no const version
	auto& grid = const_cast<Bonxai::VoxelGrid<voxel_node_t>&>(m_impl->grid);

	// Go thru all voxels:
	auto lmbdPerVoxel = [this, &bbox, &grid, &gl_obj, general_color_u,
						 general_color](
							voxel_node_t& data, const Bonxai::CoordT& coord) {
		using mrpt::img::TColor;

		// log-odds to probability:
		const double occ = 1.0 - this->l2p(data);
		const auto pt = Bonxai::CoordToPos(coord, grid.resolution);
		bbox.updateWithPoint({pt.x, pt.y, pt.z});

		if ((occ >= 0.5 && renderingOptions.generateOccupiedVoxels) ||
			(occ < 0.5 && renderingOptions.generateFreeVoxels))
		{
			mrpt::img::TColor vx_color;
			double coefc, coeft;
			switch (gl_obj.getVisualizationMode())
			{
				case COctoMapVoxels::FIXED: vx_color = general_color_u; break;

				case COctoMapVoxels::COLOR_FROM_HEIGHT:	 // not supported
					THROW_EXCEPTION(
						"COLOR_FROM_HEIGHT not supported yet for this "
						"class");
					break;

				case COctoMapVoxels::COLOR_FROM_OCCUPANCY:
					coefc = 240 * (1 - occ) + 15;
					vx_color = TColor(
						coefc * general_color.R, coefc * general_color.G,
						coefc * general_color.B, 255.0 * general_color.A);
					break;

				case COctoMapVoxels::TRANSPARENCY_FROM_OCCUPANCY:
					coeft = 255 - 510 * (1 - occ);
					if (coeft < 0) { coeft = 0; }
					vx_color = TColor(
						255 * general_color.R, 255 * general_color.G,
						255 * general_color.B, coeft);
					break;

				case COctoMapVoxels::TRANS_AND_COLOR_FROM_OCCUPANCY:
					coefc = 240 * (1 - occ) + 15;
					vx_color = TColor(
						coefc * general_color.R, coefc * general_color.G,
						coefc * general_color.B, 50);
					break;

				case COctoMapVoxels::MIXED:
					THROW_EXCEPTION("MIXED not supported yet for this class");
					break;

				default: THROW_EXCEPTION("Unknown coloring scheme!");
			}

			const size_t vx_set =
				(occ > 0.5) ? VOXEL_SET_OCCUPIED : VOXEL_SET_FREESPACE;

			gl_obj.push_back_Voxel(
				vx_set,
				COctoMapVoxels::TVoxel(
					mrpt::math::TPoint3Df(pt.x, pt.y, pt.z), grid.resolution,
					vx_color));
		}
	};	// end lambda for each voxel

	grid.forEachCell(lmbdPerVoxel);

	// if we use transparency, sort cubes by "Z" as an approximation to
	// far-to-near render ordering:
	if (gl_obj.isCubeTransparencyEnabled()) gl_obj.sort_voxels_by_z();

	// Set bounding box:
	gl_obj.setBoundingBox(bbox.min, bbox.max);
}

void CVoxelMap::TInsertionOptions::loadFromConfigFile(
	const mrpt::config::CConfigFileBase& c, const std::string& s)
{
	MRPT_LOAD_CONFIG_VAR(max_range, double, c, s);
	MRPT_LOAD_CONFIG_VAR(prob_miss, double, c, s);
	MRPT_LOAD_CONFIG_VAR(prob_hit, double, c, s);
	MRPT_LOAD_CONFIG_VAR(clamp_min, double, c, s);
	MRPT_LOAD_CONFIG_VAR(clamp_max, double, c, s);
	MRPT_LOAD_CONFIG_VAR(ray_trace_free_space, bool, c, s);
	MRPT_LOAD_CONFIG_VAR(decimation, uint64_t, c, s);
}
void CVoxelMap::TInsertionOptions::saveToConfigFile(
	mrpt::config::CConfigFileBase& c, const std::string& s) const
{
	MRPT_SAVE_CONFIG_VAR(max_range, c, s);
	MRPT_SAVE_CONFIG_VAR(prob_miss, c, s);
	MRPT_SAVE_CONFIG_VAR(prob_hit, c, s);
	MRPT_SAVE_CONFIG_VAR(clamp_min, c, s);
	MRPT_SAVE_CONFIG_VAR(clamp_max, c, s);
	MRPT_SAVE_CONFIG_VAR(ray_trace_free_space, c, s);
	MRPT_SAVE_CONFIG_VAR(decimation, c, s);
}

void CVoxelMap::TInsertionOptions::writeToStream(
	mrpt::serialization::CArchive& out) const
{
	const uint8_t version = 0;
	out << version;

	out << max_range << prob_miss << prob_hit << clamp_min << clamp_max;
	out << ray_trace_free_space << decimation;
}

void CVoxelMap::TInsertionOptions::readFromStream(
	mrpt::serialization::CArchive& in)
{
	const uint8_t version = in.ReadAs<uint8_t>();
	switch (version)
	{
		case 0:
			in >> max_range >> prob_miss >> prob_hit >> clamp_min >> clamp_max;
			in >> ray_trace_free_space >> decimation;
			break;
		default: MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	}
}

void CVoxelMap::TRenderingOptions::writeToStream(
	mrpt::serialization::CArchive& out) const
{
	const uint8_t version = 0;
	out << version;

	out << generateOccupiedVoxels << visibleOccupiedVoxels;
	out << generateFreeVoxels << visibleFreeVoxels;
}

void CVoxelMap::TRenderingOptions::readFromStream(
	mrpt::serialization::CArchive& in)
{
	const uint8_t version = in.ReadAs<uint8_t>();
	switch (version)
	{
		case 0:
			in >> generateOccupiedVoxels >> visibleOccupiedVoxels;
			in >> generateFreeVoxels >> visibleFreeVoxels;
			break;
		default: MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	}
}

void CVoxelMap::TLikelihoodOptions::loadFromConfigFile(
	const mrpt::config::CConfigFileBase& c, const std::string& s)
{
	MRPT_LOAD_CONFIG_VAR(decimation, int, c, s);
}
void CVoxelMap::TLikelihoodOptions::saveToConfigFile(
	mrpt::config::CConfigFileBase& c, const std::string& s) const
{
	MRPT_SAVE_CONFIG_VAR(decimation, c, s);
}

void CVoxelMap::TLikelihoodOptions::writeToStream(
	mrpt::serialization::CArchive& out) const
{
	out << decimation;
}

void CVoxelMap::TLikelihoodOptions::readFromStream(
	mrpt::serialization::CArchive& in)
{
	in >> decimation;
}

void CVoxelMap::internal_clear()
{
	// Is this enough?
	m_impl->grid.root_map.clear();
}

void CVoxelMap::updateVoxel(
	const double x, const double y, const double z, bool occupied)
{
	voxel_node_t* cell = m_impl->accessor.value(
		Bonxai::PosToCoord({x, y, z}, m_impl->grid.inv_resolution),
		true /*create*/);
	if (!cell) return;	// should never happen?

	if (occupied)
	{
		const voxel_node_t logodd_observation_occupied =
			std::max<voxel_node_t>(1, p2l(insertionOptions.prob_hit));
		const voxel_node_t logodd_thres_occupied =
			p2l(1.0 - insertionOptions.clamp_max);

		updateCell_fast_occupied(
			cell, logodd_observation_occupied, logodd_thres_occupied);
	}
	else
	{
		const voxel_node_t logodd_observation_free =
			std::max<voxel_node_t>(1, p2l(insertionOptions.prob_miss));
		const voxel_node_t logodd_thres_free =
			p2l(1.0 - insertionOptions.clamp_min);

		updateCell_fast_free(cell, logodd_observation_free, logodd_thres_free);
	}
}

bool CVoxelMap::getPointOccupancy(
	const double x, const double y, const double z,
	double& prob_occupancy) const
{
	voxel_node_t* cell = m_impl->accessor.value(
		Bonxai::PosToCoord({x, y, z}, m_impl->grid.inv_resolution),
		false /*create*/);

	if (!cell) return false;

	prob_occupancy = 1.0 - l2p(*cell);
	return true;
}

void CVoxelMap::insertPointCloudAsRays(
	const mrpt::maps::CPointsMap& pts, const mrpt::math::TPoint3D& sensorPt)
{
	const voxel_node_t logodd_observation_occupied =
		std::max<voxel_node_t>(1, p2l(insertionOptions.prob_hit));
	const voxel_node_t logodd_thres_occupied =
		p2l(1.0 - insertionOptions.clamp_max);

	const auto& xs = pts.getPointsBufferRef_x();
	const auto& ys = pts.getPointsBufferRef_y();
	const auto& zs = pts.getPointsBufferRef_z();

	const auto maxSqrDist = mrpt::square(insertionOptions.max_range);

	// Starting cell index at sensor pose:
	Bonxai::CoordT sensorCoord = Bonxai::PosToCoord(
		{sensorPt.x, sensorPt.y, sensorPt.z}, m_impl->grid.inv_resolution);

	// Use fixed comma for the ray tracing direction:
	constexpr unsigned int FRBITS = 9;

	const voxel_node_t logodd_observation_free =
		std::max<voxel_node_t>(1, p2l(insertionOptions.prob_miss));
	const voxel_node_t logodd_thres_free =
		p2l(1.0 - insertionOptions.clamp_min);

	// for each ray:
	for (size_t i = 0; i < xs.size(); i += insertionOptions.decimation)
	{
		if (insertionOptions.max_range > 0 &&
			mrpt::math::TPoint3D(xs[i], ys[i], zs[i]).sqrNorm() > maxSqrDist)
			continue;  // skip

		const Bonxai::CoordT endCoord = Bonxai::PosToCoord(
			{xs[i], ys[i], zs[i]}, m_impl->grid.inv_resolution);

		// jump in discrete steps from sensorCoord to endCoord:
		// Use "fractional integers" to approximate float operations
		//  during the ray tracing:
		const Bonxai::CoordT Ac = endCoord - sensorCoord;

		uint32_t Acx_ = std::abs(Ac.x);
		uint32_t Acy_ = std::abs(Ac.y);
		uint32_t Acz_ = std::abs(Ac.z);

		const auto nStepsRay = std::max(Acx_, std::max(Acy_, Acz_));
		if (!nStepsRay) continue;  // May be...

		// Integers store "float values * 128"
		float N_1 = 1.0f / nStepsRay;  // Avoid division twice.

		// Increments at each raytracing step:
		int frAcx = (Ac.x < 0 ? -1 : +1) * round((Acx_ << FRBITS) * N_1);
		int frAcy = (Ac.y < 0 ? -1 : +1) * round((Acy_ << FRBITS) * N_1);
		int frAcz = (Ac.z < 0 ? -1 : +1) * round((Acz_ << FRBITS) * N_1);

		int frCX = sensorCoord.x << FRBITS;
		int frCY = sensorCoord.y << FRBITS;
		int frCZ = sensorCoord.z << FRBITS;

		// free space ray:
		for (unsigned int nStep = 0; nStep < nStepsRay; nStep++)
		{
			if (voxel_node_t* cell = m_impl->accessor.value(
					{frCX >> FRBITS, frCY >> FRBITS, frCZ >> FRBITS},
					true /*create*/);
				cell)
			{
				updateCell_fast_free(
					cell, logodd_observation_free, logodd_thres_free);
			}

			frCX += frAcx;
			frCY += frAcy;
			frCZ += frAcz;
		}

		// and occupied end point:
		if (voxel_node_t* cell =
				m_impl->accessor.value(endCoord, true /*create*/);
			cell)
		{
			updateCell_fast_occupied(
				cell, logodd_observation_occupied, logodd_thres_occupied);
		}
	}  // for each point/ray
}

void CVoxelMap::insertPointCloudAsEndPoints(const mrpt::maps::CPointsMap& pts)
{
	const voxel_node_t logodd_observation_occupied =
		std::max<voxel_node_t>(1, p2l(insertionOptions.prob_hit));
	const voxel_node_t logodd_thres_occupied =
		p2l(1.0 - insertionOptions.clamp_max);

	const auto& xs = pts.getPointsBufferRef_x();
	const auto& ys = pts.getPointsBufferRef_y();
	const auto& zs = pts.getPointsBufferRef_z();

	const auto maxSqrDist = mrpt::square(insertionOptions.max_range);

	for (size_t i = 0; i < xs.size(); i += insertionOptions.decimation)
	{
		if (insertionOptions.max_range > 0 &&
			mrpt::math::TPoint3D(xs[i], ys[i], zs[i]).sqrNorm() > maxSqrDist)
			continue;  // skip

		voxel_node_t* cell = m_impl->accessor.value(
			Bonxai::PosToCoord(
				{xs[i], ys[i], zs[i]}, m_impl->grid.inv_resolution),
			true /*create*/);
		if (!cell) continue;  // should never happen?

		updateCell_fast_occupied(
			cell, logodd_observation_occupied, logodd_thres_occupied);
	}
}
