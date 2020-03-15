/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "maps-precomp.h"  // Precomp header

#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/maps/COccupancyGridMap3D.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/opengl/COctoMapVoxels.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt;
using namespace mrpt::maps;

//  =========== Begin of Map definition ============
MAP_DEFINITION_REGISTER(
	"mrpt::maps::COccupancyGridMap3D", mrpt::maps::COccupancyGridMap3D)

COccupancyGridMap3D::TMapDefinition::TMapDefinition() = default;

void COccupancyGridMap3D::TMapDefinition::loadFromConfigFile_map_specific(
	const mrpt::config::CConfigFileBase& source, const std::string& sect)
{
	using namespace std::string_literals;

	// [<sect>+"_creationOpts"]
	const auto sSectCreation = sect + "_creationOpts"s;
	MRPT_LOAD_CONFIG_VAR(min_x, float, source, sSectCreation);
	MRPT_LOAD_CONFIG_VAR(max_x, float, source, sSectCreation);
	MRPT_LOAD_CONFIG_VAR(min_y, float, source, sSectCreation);
	MRPT_LOAD_CONFIG_VAR(max_y, float, source, sSectCreation);
	MRPT_LOAD_CONFIG_VAR(min_z, float, source, sSectCreation);
	MRPT_LOAD_CONFIG_VAR(max_z, float, source, sSectCreation);
	MRPT_LOAD_CONFIG_VAR(resolution, float, source, sSectCreation);

	// [<sectionName>+"_occupancyGrid_##_insertOpts"]
	insertionOpts.loadFromConfigFile(source, sect + "_insertOpts"s);

	// [<sectionName>+"_occupancyGrid_##_likelihoodOpts"]
	likelihoodOpts.loadFromConfigFile(source, sect + "_likelihoodOpts"s);
}

void COccupancyGridMap3D::TMapDefinition::dumpToTextStream_map_specific(
	std::ostream& out) const
{
	LOADABLEOPTS_DUMP_VAR(min_x, float);
	LOADABLEOPTS_DUMP_VAR(max_x, float);
	LOADABLEOPTS_DUMP_VAR(min_y, float);
	LOADABLEOPTS_DUMP_VAR(max_y, float);
	LOADABLEOPTS_DUMP_VAR(min_z, float);
	LOADABLEOPTS_DUMP_VAR(max_z, float);
	LOADABLEOPTS_DUMP_VAR(resolution, float);

	this->insertionOpts.dumpToTextStream(out);
	this->likelihoodOpts.dumpToTextStream(out);
}

mrpt::maps::CMetricMap* COccupancyGridMap3D::internal_CreateFromMapDefinition(
	const mrpt::maps::TMetricMapInitializer& _def)
{
	auto& def = dynamic_cast<const COccupancyGridMap3D::TMapDefinition&>(_def);
	auto* obj = new COccupancyGridMap3D(
		mrpt::math::TPoint3D(def.min_x, def.min_y, def.min_z),
		mrpt::math::TPoint3D(def.max_x, def.max_y, def.max_z), def.resolution);
	obj->insertionOptions = def.insertionOpts;
	obj->likelihoodOptions = def.likelihoodOpts;
	return obj;
}
//  =========== End of Map definition Block =========

IMPLEMENTS_SERIALIZABLE(COccupancyGridMap3D, CMetricMap, mrpt::maps)

// Static lookup tables for log-odds
static CLogOddsGridMapLUT<COccupancyGridMap3D::voxelType> logodd_lut;

CLogOddsGridMapLUT<COccupancyGridMap3D::voxelType>&
	COccupancyGridMap3D::get_logodd_lut()
{
	return logodd_lut;
}

COccupancyGridMap3D::COccupancyGridMap3D(
	const mrpt::math::TPoint3D& corner_min,
	const mrpt::math::TPoint3D& corner_max, float resolution)
{
	MRPT_START
	setSize(corner_min, corner_max, resolution, 0.5f);
	MRPT_END
}

void COccupancyGridMap3D::setSize(
	const mrpt::math::TPoint3D& cmin, const mrpt::math::TPoint3D& cmax,
	double res, float default_value)
{
	MRPT_START

	ASSERT_ABOVE_(res, 0.0);
	ASSERT_ABOVE_(cmax.x, cmin.x);
	ASSERT_ABOVE_(cmax.y, cmin.y);
	ASSERT_ABOVE_(cmax.z, cmin.z);
	ASSERT_ABOVEEQ_(default_value, 0.0f);
	ASSERT_BELOWEQ_(default_value, 1.0f);

	const auto def_value = p2l(default_value);
	m_grid.setSize(
		cmin.x, cmax.x, cmin.y, cmax.y, cmin.z, cmax.z, res, res, &def_value);

	// m_likelihoodCacheOutDated = true;
	m_is_empty = true;

	MRPT_END
}

void COccupancyGridMap3D::resizeGrid(
	const mrpt::math::TPoint3D& cmin, const mrpt::math::TPoint3D& cmax,
	float new_cells_default_value)
{
	MRPT_START

	const auto def_value = p2l(new_cells_default_value);
	const double additionalMargin = .0;
	m_grid.resize(
		cmin.x, cmax.x, cmin.y, cmax.y, cmin.z, cmax.z, def_value,
		additionalMargin);

	// m_likelihoodCacheOutDated = true;
	m_is_empty = true;

	MRPT_END
}

void COccupancyGridMap3D::internal_clear()
{
	TMapDefinition md;

	setSize(
		mrpt::math::TPoint3D(md.min_x, md.min_y, md.min_z),
		mrpt::math::TPoint3D(md.max_x, md.max_y, md.max_z),
		m_grid.getResolutionXY());

	// m_likelihoodCacheOutDated = true;
	m_is_empty = true;
}

void COccupancyGridMap3D::fill(float default_value)
{
	const voxelType defValue = p2l(default_value);
	m_grid.fill(defValue);
	// m_likelihoodCacheOutDated = true;
}

void COccupancyGridMap3D::updateCell(int x, int y, int z, float v)
{
	if (m_grid.isOutOfBounds(x, y, z)) return;

	// Get the current contents of the cell:
	auto* cp = m_grid.cellByIndex(x, y, z);
	ASSERT_(cp != nullptr);
	voxelType& theCell = *cp;

	// Compute the new Bayesian-fused value of the cell:
	// The observation: will be >0 for free, <0 for occupied.
	const voxelType obs = p2l(v);
	if (obs > 0)
	{
		// Saturate
		if (theCell > (CLogOddsGridMap3D<voxelType>::CELLTYPE_MAX - obs))
			theCell = CLogOddsGridMap3D<voxelType>::CELLTYPE_MAX;
		else
			theCell += obs;
	}
	else
	{
		// Saturate
		if (theCell < (CLogOddsGridMap3D<voxelType>::CELLTYPE_MIN - obs))
			theCell = CLogOddsGridMap3D<voxelType>::CELLTYPE_MIN;
		else
			theCell += obs;
	}
}

void COccupancyGridMap3D::determineMatching2D(
	const mrpt::maps::CMetricMap* otherMap2,
	const mrpt::poses::CPose2D& otherMapPose_,
	mrpt::tfest::TMatchingPairList& correspondences,
	const mrpt::maps::TMatchingParams& params,
	mrpt::maps::TMatchingExtraResults& extraResults) const
{
	MRPT_START

	THROW_EXCEPTION("Implement me!");

	MRPT_END
}

bool COccupancyGridMap3D::isEmpty() const { return m_is_empty; }

float COccupancyGridMap3D::compute3DMatchingRatio(
	const mrpt::maps::CMetricMap* otherMap,
	const mrpt::poses::CPose3D& otherMapPose,
	const TMatchingRatioParams& params) const
{
	MRPT_UNUSED_PARAM(otherMap);
	MRPT_UNUSED_PARAM(otherMapPose);
	MRPT_UNUSED_PARAM(params);
	THROW_EXCEPTION("Implement me!");
	return 0;
}

void COccupancyGridMap3D::getAsOctoMapVoxels(
	mrpt::opengl::COctoMapVoxels& gl_obj) const
{
	MRPT_START

	using mrpt::img::TColor;
	using mrpt::img::TColorf;
	using namespace mrpt::opengl;

	const size_t N = m_grid.getSizeX() * m_grid.getSizeY() * m_grid.getSizeZ();
	const TColorf general_color = gl_obj.getColor();
	const TColor general_color_u = general_color.asTColor();

	gl_obj.clear();
	gl_obj.resizeVoxelSets(2);  // 2 sets of voxels: occupied & free
	if (renderingOptions.generateGridLines) gl_obj.reserveGridCubes(N);

	gl_obj.showVoxels(
		mrpt::opengl::VOXEL_SET_OCCUPIED,
		renderingOptions.visibleOccupiedVoxels);
	gl_obj.showVoxels(
		mrpt::opengl::VOXEL_SET_FREESPACE, renderingOptions.visibleFreeVoxels);

	gl_obj.reserveVoxels(mrpt::opengl::VOXEL_SET_OCCUPIED, N / 4);
	gl_obj.reserveVoxels(mrpt::opengl::VOXEL_SET_FREESPACE, N / 4);

	const mrpt::math::TPoint3D bbmin(
		m_grid.getXMin(), m_grid.getYMin(), m_grid.getZMin());
	const mrpt::math::TPoint3D bbmax(
		m_grid.getXMax(), m_grid.getYMax(), m_grid.getZMax());
	const float inv_dz = 1.0f / d2f(bbmax.z - bbmin.z + 0.01f);
	const double L = 0.5 * m_grid.getResolutionZ();

	for (size_t cz = 0; cz < m_grid.getSizeZ(); cz++)
	{
		// voxel center coordinates:
		const double z = m_grid.idx2z(cz) + m_grid.getResolutionZ() * 0.5;
		for (size_t cy = 0; cy < m_grid.getSizeY(); cy++)
		{
			const double y = m_grid.idx2y(cy) + m_grid.getResolutionXY() * 0.5;
			for (size_t cx = 0; cx < m_grid.getSizeX(); cx++)
			{
				const double x =
					m_grid.idx2x(cx) + m_grid.getResolutionXY() * 0.5;
				const float occ = 1.0f - this->getCellFreeness(cx, cy, cz);
				const bool is_occupied = occ > 0.501f;
				const bool is_free = occ < 0.499f;
				if ((is_occupied && renderingOptions.generateOccupiedVoxels) ||
					(is_free && renderingOptions.generateFreeVoxels))
				{
					mrpt::img::TColor vx_color;
					float coefc, coeft;
					switch (gl_obj.getVisualizationMode())
					{
						case COctoMapVoxels::FIXED:
							vx_color = general_color_u;
							break;
						case COctoMapVoxels::COLOR_FROM_HEIGHT:
							coefc = 255 * inv_dz * d2f(z - bbmin.z);
							vx_color = TColor(
								f2u8(coefc * general_color.R),
								f2u8(coefc * general_color.G),
								f2u8(coefc * general_color.B),
								f2u8(255 * general_color.A));
							break;

						case COctoMapVoxels::COLOR_FROM_OCCUPANCY:
							coefc = 240 * (1 - occ) + 15;
							vx_color = TColor(
								f2u8(coefc * general_color.R),
								f2u8(coefc * general_color.G),
								f2u8(coefc * general_color.B),
								f2u8(255 * general_color.A));
							break;

						case COctoMapVoxels::TRANSPARENCY_FROM_OCCUPANCY:
							coeft = 255 - 510 * (1 - occ);
							if (coeft < 0)
							{
								coeft = 0;
							}
							vx_color = general_color.asTColor();
							vx_color.A = mrpt::round(coeft);
							break;

						case COctoMapVoxels::TRANS_AND_COLOR_FROM_OCCUPANCY:
							coefc = 240 * (1 - occ) + 15;
							vx_color = TColor(
								f2u8(coefc * general_color.R),
								f2u8(coefc * general_color.G),
								f2u8(coefc * general_color.B), 50);
							break;

						case COctoMapVoxels::MIXED:
							coefc = d2f(255 * inv_dz * (z - bbmin.z));
							coeft = d2f(255 - 510 * (1 - occ));
							if (coeft < 0)
							{
								coeft = 0;
							}
							vx_color = TColor(
								f2u8(coefc * general_color.R),
								f2u8(coefc * general_color.G),
								f2u8(coefc * general_color.B),
								static_cast<uint8_t>(coeft));
							break;

						default:
							THROW_EXCEPTION("Unknown coloring scheme!");
					}

					const size_t vx_set =
						is_occupied ? VOXEL_SET_OCCUPIED : VOXEL_SET_FREESPACE;

					gl_obj.push_back_Voxel(
						vx_set,
						COctoMapVoxels::TVoxel(
							mrpt::math::TPoint3D(x, y, z), 2 * L, vx_color));
				}

				if (renderingOptions.generateGridLines)
				{
					// Not leaf-nodes:
					const mrpt::math::TPoint3D pt_min(x - L, y - L, z - L);
					const mrpt::math::TPoint3D pt_max(x + L, y + L, z + L);
					gl_obj.push_back_GridCube(
						COctoMapVoxels::TGridCube(pt_min, pt_max));
				}
			}
		}
	}  // end for each voxel

	// if we use transparency, sort cubes by "Z" as an approximation to
	// far-to-near render ordering:
	if (gl_obj.isCubeTransparencyEnabled()) gl_obj.sort_voxels_by_z();

	// Set bounding box:
	gl_obj.setBoundingBox(bbmin, bbmax);

	MRPT_END
}

void COccupancyGridMap3D::getAs3DObject(
	mrpt::opengl::CSetOfObjects::Ptr& outObj) const
{
	auto gl_obj = mrpt::opengl::COctoMapVoxels::Create();
	this->getAsOctoMapVoxels(*gl_obj);
	outObj->insert(gl_obj);
}

uint8_t COccupancyGridMap3D::serializeGetVersion() const { return 0; }
void COccupancyGridMap3D::serializeTo(mrpt::serialization::CArchive& out) const
{
// Version 2: Save OCCUPANCY_GRIDMAP_CELL_SIZE_8BITS/16BITS
#ifdef OCCUPANCY_GRIDMAP_CELL_SIZE_8BITS
	out << uint8_t(8);
#else
	out << uint8_t(16);
#endif

	// Save grid dimensions:
	m_grid.dyngridcommon_writeToStream(out);

#ifdef OCCUPANCY_GRIDMAP_CELL_SIZE_8BITS
	out.WriteBuffer
#else
	out.WriteBufferFixEndianness
#endif
		(m_grid.cellByIndex(0, 0, 0), sizeof(cell_t) * m_grid.getSizeX() *
										  m_grid.getSizeY() *
										  m_grid.getSizeZ());

	// insertionOptions:
	out << insertionOptions.maxDistanceInsertion
		<< insertionOptions.maxOccupancyUpdateCertainty
		<< insertionOptions.maxFreenessUpdateCertainty
		<< insertionOptions.decimation << insertionOptions.decimation_3d_range;

	// Likelihood:
	out.WriteAs<int32_t>(likelihoodOptions.likelihoodMethod);
	out << likelihoodOptions.LF_stdHit << likelihoodOptions.LF_zHit
		<< likelihoodOptions.LF_zRandom << likelihoodOptions.LF_maxRange
		<< likelihoodOptions.LF_decimation
		<< likelihoodOptions.LF_maxCorrsDistance
		<< likelihoodOptions.LF_useSquareDist
		<< likelihoodOptions.rayTracing_decimation
		<< likelihoodOptions.rayTracing_stdHit;

	out << genericMapParams;

	renderingOptions.writeToStream(out);
}

void COccupancyGridMap3D::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	m_is_empty = false;

	switch (version)
	{
		case 0:
		{
			uint8_t bitsPerCellStream;
			in >> bitsPerCellStream;

#ifdef OCCUPANCY_GRIDMAP_CELL_SIZE_8BITS
			ASSERT_(bitsPerCellStream == 8);
#else
			ASSERT_(bitsPerCellStream == 16);
#endif

			// Save grid dimensions:
			m_grid.dyngridcommon_readFromStream(in);

#ifdef OCCUPANCY_GRIDMAP_CELL_SIZE_8BITS
			in.ReadBuffer
#else
			in.ReadBufferFixEndianness
#endif
				(m_grid.cellByIndex(0, 0, 0),
				 sizeof(cell_t) * m_grid.getSizeX() * m_grid.getSizeY() *
					 m_grid.getSizeZ());

			// insertionOptions:
			in >> insertionOptions.maxDistanceInsertion >>
				insertionOptions.maxOccupancyUpdateCertainty >>
				insertionOptions.maxFreenessUpdateCertainty >>
				insertionOptions.decimation >>
				insertionOptions.decimation_3d_range;

			// Likelihood:
			in.ReadAsAndCastTo<int32_t, TLikelihoodMethod>(
				likelihoodOptions.likelihoodMethod);

			in >> likelihoodOptions.LF_stdHit >> likelihoodOptions.LF_zHit >>
				likelihoodOptions.LF_zRandom >> likelihoodOptions.LF_maxRange >>
				likelihoodOptions.LF_decimation >>
				likelihoodOptions.LF_maxCorrsDistance >>
				likelihoodOptions.LF_useSquareDist >>
				likelihoodOptions.rayTracing_decimation >>
				likelihoodOptions.rayTracing_stdHit;

			in >> genericMapParams;

			renderingOptions.readFromStream(in);
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
}

void COccupancyGridMap3D::TRenderingOptions::writeToStream(
	mrpt::serialization::CArchive& out) const
{
	const int8_t version = 0;
	out << version;
	out << generateGridLines << generateOccupiedVoxels << visibleOccupiedVoxels
		<< generateFreeVoxels << visibleFreeVoxels;
}

void COccupancyGridMap3D::TRenderingOptions::readFromStream(
	mrpt::serialization::CArchive& in)
{
	int8_t version;
	in >> version;
	switch (version)
	{
		case 0:
		{
			in >> generateGridLines >> generateOccupiedVoxels >>
				visibleOccupiedVoxels >> generateFreeVoxels >>
				visibleFreeVoxels;
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	}
}

void COccupancyGridMap3D::saveMetricMapRepresentationToFile(
	const std::string& filNamePrefix) const
{
	using namespace std::string_literals;
	const auto fil = filNamePrefix + ".txt";
	// todo
}
