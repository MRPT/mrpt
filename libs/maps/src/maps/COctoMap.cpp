/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "maps-precomp.h"  // Precomp header

#include <mrpt/maps/COctoMap.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::img;
using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::opengl;

//  =========== Begin of Map definition ============
MAP_DEFINITION_REGISTER("COctoMap,octoMap", mrpt::maps::COctoMap)

COctoMap::TMapDefinition::TMapDefinition() : resolution(0.10) {}
void COctoMap::TMapDefinition::loadFromConfigFile_map_specific(
	const mrpt::config::CConfigFileBase& source,
	const std::string& sectionNamePrefix)
{
	// [<sectionNamePrefix>+"_creationOpts"]
	const std::string sSectCreation =
		sectionNamePrefix + string("_creationOpts");
	MRPT_LOAD_CONFIG_VAR(resolution, double, source, sSectCreation);

	insertionOpts.loadFromConfigFile(
		source, sectionNamePrefix + string("_insertOpts"));
	likelihoodOpts.loadFromConfigFile(
		source, sectionNamePrefix + string("_likelihoodOpts"));
}

void COctoMap::TMapDefinition::dumpToTextStream_map_specific(
	std::ostream& out) const
{
	LOADABLEOPTS_DUMP_VAR(resolution, double);

	this->insertionOpts.dumpToTextStream(out);
	this->likelihoodOpts.dumpToTextStream(out);
}

mrpt::maps::CMetricMap* COctoMap::internal_CreateFromMapDefinition(
	const mrpt::maps::TMetricMapInitializer& _def)
{
	const COctoMap::TMapDefinition& def =
		*dynamic_cast<const COctoMap::TMapDefinition*>(&_def);
	COctoMap* obj = new COctoMap(def.resolution);
	obj->insertionOptions = def.insertionOpts;
	obj->likelihoodOptions = def.likelihoodOpts;
	return obj;
}
//  =========== End of Map definition Block =========

IMPLEMENTS_SERIALIZABLE(COctoMap, CMetricMap, mrpt::maps)

/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
COctoMap::COctoMap(const double resolution)
	: COctoMapBase<octomap::OcTree, octomap::OcTreeNode>(resolution)
{
}

COctoMap::~COctoMap() {}

uint8_t COctoMap::serializeGetVersion() const { return 3; }
void COctoMap::serializeTo(mrpt::serialization::CArchive& out) const
{
	this->likelihoodOptions.writeToStream(out);
	this->renderingOptions.writeToStream(out);  // Added in v1
	out << genericMapParams;
	// v2->v3: remove CMemoryChunk
	std::stringstream ss;
	const_cast<octomap::OcTree*>(&m_octomap)->writeBinary(ss);
	const std::string& buf = ss.str();
	out << buf;
}

void COctoMap::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		case 1:
		case 2:
		{
			THROW_EXCEPTION("Deserialization of old versions of this class was discontinued in MRPT 1.9.9 [no CMemoryChunk]");
		}
		break;
		case 3:
		{
			this->likelihoodOptions.readFromStream(in);
			if (version >= 1) this->renderingOptions.readFromStream(in);
			if (version >= 2) in >> genericMapParams;

			this->clear();

			std::string buf;
			in >> buf;

			if (!buf.empty())
			{
				std::stringstream ss;
				ss.str(buf);
				ss.seekg(0);
				m_octomap.readBinary(ss);
			}
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

bool COctoMap::internal_insertObservation(
	const mrpt::obs::CObservation* obs, const CPose3D* robotPose)
{
	octomap::point3d sensorPt;
	octomap::Pointcloud scan;
	if (!internal_build_PointCloud_for_observation(
			obs, robotPose, sensorPt, scan))
		return false;  // Nothing to do.
	// Insert rays:
	m_octomap.insertPointCloud(
		scan, sensorPt, insertionOptions.maxrange, insertionOptions.pruning);
	return true;
}

/** Builds a renderizable representation of the octomap as a
 * mrpt::opengl::COctoMapVoxels object. */
void COctoMap::getAsOctoMapVoxels(mrpt::opengl::COctoMapVoxels& gl_obj) const
{
	// Go thru all voxels:
	// OcTreeVolume voxel; // current voxel, possibly transformed
	octomap::OcTree::tree_iterator it_end = m_octomap.end_tree();

	const unsigned char max_depth = 0;  // all
	const TColorf general_color = gl_obj.getColor();
	const TColor general_color_u(
		general_color.R * 255, general_color.G * 255, general_color.B * 255,
		general_color.A * 255);

	gl_obj.clear();
	gl_obj.reserveGridCubes(this->calcNumNodes());

	gl_obj.resizeVoxelSets(2);  // 2 sets of voxels: occupied & free

	gl_obj.showVoxels(
		VOXEL_SET_OCCUPIED, renderingOptions.visibleOccupiedVoxels);
	gl_obj.showVoxels(VOXEL_SET_FREESPACE, renderingOptions.visibleFreeVoxels);

	const size_t nLeafs = this->getNumLeafNodes();
	gl_obj.reserveVoxels(VOXEL_SET_OCCUPIED, nLeafs);
	gl_obj.reserveVoxels(VOXEL_SET_FREESPACE, nLeafs);

	double xmin, xmax, ymin, ymax, zmin, zmax, inv_dz;
	this->getMetricMin(xmin, ymin, zmin);
	this->getMetricMax(xmax, ymax, zmax);
	inv_dz = 1 / (zmax - zmin + 0.01);

	for (octomap::OcTree::tree_iterator it = m_octomap.begin_tree(max_depth);
		 it != it_end; ++it)
	{
		const octomap::point3d vx_center = it.getCoordinate();
		const double vx_length = it.getSize();
		const double L = 0.5 * vx_length;

		if (it.isLeaf())
		{
			// voxels for leaf nodes
			const double occ = it->getOccupancy();
			if ((occ >= 0.5 && renderingOptions.generateOccupiedVoxels) ||
				(occ < 0.5 && renderingOptions.generateFreeVoxels))
			{
				mrpt::img::TColor vx_color;
				double coefc, coeft;
				switch (gl_obj.getVisualizationMode())
				{
					case COctoMapVoxels::FIXED:
						vx_color = general_color_u;
						break;
					case COctoMapVoxels::COLOR_FROM_HEIGHT:
						coefc = 255 * inv_dz * (vx_center.z() - zmin);
						vx_color = TColor(
							coefc * general_color.R, coefc * general_color.G,
							coefc * general_color.B, 255.0 * general_color.A);
						break;

					case COctoMapVoxels::COLOR_FROM_OCCUPANCY:
						coefc = 240 * (1 - occ) + 15;
						vx_color = TColor(
							coefc * general_color.R, coefc * general_color.G,
							coefc * general_color.B, 255.0 * general_color.A);
						break;

					case COctoMapVoxels::TRANSPARENCY_FROM_OCCUPANCY:
						coeft = 255 - 510 * (1 - occ);
						if (coeft < 0)
						{
							coeft = 0;
						}
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
						coefc = 255 * inv_dz * (vx_center.z() - zmin);
						coeft = 255 - 510 * (1 - occ);
						if (coeft < 0)
						{
							coeft = 0;
						}
						vx_color = TColor(
							coefc * general_color.R, coefc * general_color.G,
							coefc * general_color.B, coeft);
						break;

					default:
						THROW_EXCEPTION("Unknown coloring scheme!");
				}

				const size_t vx_set = (m_octomap.isNodeOccupied(*it))
										  ? VOXEL_SET_OCCUPIED
										  : VOXEL_SET_FREESPACE;

				gl_obj.push_back_Voxel(
					vx_set,
					COctoMapVoxels::TVoxel(
						TPoint3D(vx_center.x(), vx_center.y(), vx_center.z()),
						vx_length, vx_color));
			}
		}
		else if (renderingOptions.generateGridLines)
		{
			// Not leaf-nodes:
			const mrpt::math::TPoint3D pt_min(
				vx_center.x() - L, vx_center.y() - L, vx_center.z() - L);
			const mrpt::math::TPoint3D pt_max(
				vx_center.x() + L, vx_center.y() + L, vx_center.z() + L);
			gl_obj.push_back_GridCube(
				COctoMapVoxels::TGridCube(pt_min, pt_max));
		}
	}  // end for each voxel

	// if we use transparency, sort cubes by "Z" as an approximation to
	// far-to-near render ordering:
	if (gl_obj.isCubeTransparencyEnabled()) gl_obj.sort_voxels_by_z();

	// Set bounding box:
	{
		mrpt::math::TPoint3D bbmin, bbmax;
		this->getMetricMin(bbmin.x, bbmin.y, bbmin.z);
		this->getMetricMax(bbmax.x, bbmax.y, bbmax.z);
		gl_obj.setBoundingBox(bbmin, bbmax);
	}
}
