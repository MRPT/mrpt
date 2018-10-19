/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "maps-precomp.h"  // Precomp header

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>

#include <mrpt/maps/CColouredOctoMap.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>

#include <mrpt/opengl/COctoMapVoxels.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CPointCloudColoured.h>

#include <mrpt/system/filesystem.h>
#include <sstream>
#include <mrpt/io/CFileOutputStream.h>

#include "COctoMapBase_impl.h"

// Explicit instantiation:
template class mrpt::maps::COctoMapBase<
	octomap::ColorOcTree, octomap::ColorOcTreeNode>;

using namespace std;
using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace mrpt::img;
using namespace mrpt::opengl;
using namespace mrpt::math;

//  =========== Begin of Map definition ============
MAP_DEFINITION_REGISTER(
	"CColouredOctoMap,colourOctoMap,colorOctoMap", mrpt::maps::CColouredOctoMap)

CColouredOctoMap::TMapDefinition::TMapDefinition() = default;
void CColouredOctoMap::TMapDefinition::loadFromConfigFile_map_specific(
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

void CColouredOctoMap::TMapDefinition::dumpToTextStream_map_specific(
	std::ostream& out) const
{
	LOADABLEOPTS_DUMP_VAR(resolution, double);

	this->insertionOpts.dumpToTextStream(out);
	this->likelihoodOpts.dumpToTextStream(out);
}

mrpt::maps::CMetricMap* CColouredOctoMap::internal_CreateFromMapDefinition(
	const mrpt::maps::TMetricMapInitializer& _def)
{
	const CColouredOctoMap::TMapDefinition& def =
		*dynamic_cast<const CColouredOctoMap::TMapDefinition*>(&_def);
	auto* obj = new CColouredOctoMap(def.resolution);
	obj->insertionOptions = def.insertionOpts;
	obj->likelihoodOptions = def.likelihoodOpts;
	return obj;
}
//  =========== End of Map definition Block =========

IMPLEMENTS_SERIALIZABLE(CColouredOctoMap, CMetricMap, mrpt::maps)

CColouredOctoMap::CColouredOctoMap(const double resolution)
	: COctoMapBase<octomap::ColorOcTree, octomap::ColorOcTreeNode>(resolution)

{
}

CColouredOctoMap::~CColouredOctoMap() = default;
uint8_t CColouredOctoMap::serializeGetVersion() const { return 3; }
void CColouredOctoMap::serializeTo(mrpt::serialization::CArchive& out) const
{
	this->likelihoodOptions.writeToStream(out);
	this->renderingOptions.writeToStream(out);  // Added in v1
	out << genericMapParams;  // v2

	// v2->v3: remove CMemoryChunk
	std::stringstream ss;
	m_impl->m_octomap.writeBinaryConst(ss);
	const std::string& buf = ss.str();
	out << buf;
}

void CColouredOctoMap::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		case 1:
		case 2:
		{
			THROW_EXCEPTION(
				"Deserialization of old versions of this class was "
				"discontinued in MRPT 1.9.9 [no CMemoryChunk]");
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
				m_impl->m_octomap.readBinary(ss);
			}
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

/*---------------------------------------------------------------
				insertObservation
 ---------------------------------------------------------------*/
bool CColouredOctoMap::internal_insertObservation(
	const mrpt::obs::CObservation* obs, const CPose3D* robotPose)
{
	octomap::point3d sensorPt;
	octomap::Pointcloud scan;

	CPose3D robotPose3D;
	if (robotPose)  // Default values are (0,0,0)
		robotPose3D = (*robotPose);

	if (IS_CLASS(obs, CObservation2DRangeScan))
	{
		/********************************************************************
				OBSERVATION TYPE: CObservation2DRangeScan
		********************************************************************/
		const auto* o = static_cast<const CObservation2DRangeScan*>(obs);

		// Build a points-map representation of the points from the scan
		// (coordinates are wrt the robot base)

		// Sensor_pose = robot_pose (+) sensor_pose_on_robot
		CPose3D sensorPose(UNINITIALIZED_POSE);
		sensorPose.composeFrom(robotPose3D, o->sensorPose);
		sensorPt =
			octomap::point3d(sensorPose.x(), sensorPose.y(), sensorPose.z());

		const auto* scanPts = o->buildAuxPointsMap<mrpt::maps::CPointsMap>();
		const size_t nPts = scanPts->size();

		// Transform 3D point cloud:
		scan.reserve(nPts);

		mrpt::math::TPoint3Df pt;
		for (size_t i = 0; i < nPts; i++)
		{
			// Load the next point:
			scanPts->getPointFast(i, pt.x, pt.y, pt.z);

			// Translation:
			double gx, gy, gz;
			robotPose3D.composePoint(pt.x, pt.y, pt.z, gx, gy, gz);

			// Add to this map:
			scan.push_back(gx, gy, gz);
		}

		// Insert rays:
		m_impl->m_octomap.insertPointCloud(
			scan, sensorPt, insertionOptions.maxrange,
			insertionOptions.pruning);
		return true;
	}
	else if (IS_CLASS(obs, CObservation3DRangeScan))
	{
		/********************************************************************
				OBSERVATION TYPE: CObservation3DRangeScan
		********************************************************************/
		const auto* o = static_cast<const CObservation3DRangeScan*>(obs);

		o->load();  // Just to make sure the points are loaded from an external
		// source, if that's the case...

		// Project 3D points & color:
		mrpt::opengl::CPointCloudColoured::Ptr pts =
			mrpt::make_aligned_shared<mrpt::opengl::CPointCloudColoured>();
		T3DPointsProjectionParams proj_params;
		proj_params.PROJ3D_USE_LUT = true;
		proj_params.robotPoseInTheWorld = robotPose;
		const_cast<CObservation3DRangeScan*>(o)
			->project3DPointsFromDepthImageInto(*pts, proj_params);

		// Sensor_pose = robot_pose (+) sensor_pose_on_robot
		CPose3D sensorPose(UNINITIALIZED_POSE);
		sensorPose.composeFrom(robotPose3D, o->sensorPose);
		sensorPt =
			octomap::point3d(sensorPose.x(), sensorPose.y(), sensorPose.z());

		const size_t sizeRangeScan = pts->size();
		scan.reserve(sizeRangeScan);

		for (size_t i = 0; i < sizeRangeScan; i++)
		{
			const mrpt::opengl::CPointCloudColoured::TPointColour& pt =
				pts->getPoint(i);

			// Add to this map:
			if (pt.x != 0 || pt.y != 0 || pt.z != 0)
				scan.push_back(pt.x, pt.y, pt.z);
		}

		// Insert rays:
		octomap::KeySet free_cells, occupied_cells;
		m_impl->m_octomap.computeUpdate(
			scan, sensorPt, free_cells, occupied_cells,
			insertionOptions.maxrange);

		// insert data into tree  -----------------------
		for (const auto& free_cell : free_cells)
		{
			m_impl->m_octomap.updateNode(free_cell, false, false);
		}
		for (const auto& occupied_cell : occupied_cells)
		{
			m_impl->m_octomap.updateNode(occupied_cell, true, false);
		}

		// Update color -----------------------
		const float colF2B = 255.0f;
		for (size_t i = 0; i < sizeRangeScan; i++)
		{
			const mrpt::opengl::CPointCloudColoured::TPointColour& pt =
				pts->getPoint(i);

			// Add to this map:
			if (pt.x != 0 || pt.y != 0 || pt.z != 0)
				this->updateVoxelColour(
					pt.x, pt.y, pt.z, uint8_t(pt.R * colF2B),
					uint8_t(pt.G * colF2B), uint8_t(pt.B * colF2B));
		}

		// TODO: does pruning make sense if we used "lazy_eval"?
		if (insertionOptions.pruning) m_impl->m_octomap.prune();

		return true;
	}

	return false;
}

/** Get the RGB colour of a point
 * \return false if the point is not mapped, in which case the returned colour
 * is undefined. */
bool CColouredOctoMap::getPointColour(
	const float x, const float y, const float z, uint8_t& r, uint8_t& g,
	uint8_t& b) const
{
	octomap::OcTreeKey key;
	if (m_impl->m_octomap.coordToKeyChecked(octomap::point3d(x, y, z), key))
	{
		octomap::ColorOcTreeNode* node =
			m_impl->m_octomap.search(key, 0 /*depth*/);
		if (!node) return false;

		const octomap::ColorOcTreeNode::Color& col = node->getColor();
		r = col.r;
		g = col.g;
		b = col.b;
		return true;
	}
	else
		return false;
}

/** Manually update the colour of the voxel at (x,y,z) */
void CColouredOctoMap::updateVoxelColour(
	const double x, const double y, const double z, const uint8_t r,
	const uint8_t g, const uint8_t b)
{
	switch (m_colour_method)
	{
		case INTEGRATE:
			m_impl->m_octomap.integrateNodeColor(x, y, z, r, g, b);
			break;
		case SET:
			m_impl->m_octomap.setNodeColor(x, y, z, r, g, b);
			break;
		case AVERAGE:
			m_impl->m_octomap.averageNodeColor(x, y, z, r, g, b);
			break;
		default:
			THROW_EXCEPTION("Invalid value found for 'm_colour_method'");
	}
}

/** Builds a renderizable representation of the octomap as a
 * mrpt::opengl::COctoMapVoxels object. */
void CColouredOctoMap::getAsOctoMapVoxels(
	mrpt::opengl::COctoMapVoxels& gl_obj) const
{
	// Go thru all voxels:

	// OcTreeVolume voxel; // current voxel, possibly transformed
	octomap::ColorOcTree::tree_iterator it_end = m_impl->m_octomap.end_tree();

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

	double xmin, xmax, ymin, ymax, zmin, zmax;
	this->getMetricMin(xmin, ymin, zmin);
	this->getMetricMax(xmax, ymax, zmax);

	for (octomap::ColorOcTree::tree_iterator it =
			 m_impl->m_octomap.begin_tree(max_depth);
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
				octomap::ColorOcTreeNode::Color node_color = it->getColor();
				vx_color = TColor(node_color.r, node_color.g, node_color.b);

				const size_t vx_set = (m_impl->m_octomap.isNodeOccupied(*it))
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

void CColouredOctoMap::insertRay(
	const float end_x, const float end_y, const float end_z,
	const float sensor_x, const float sensor_y, const float sensor_z)
{
	m_impl->m_octomap.insertRay(
		octomap::point3d(sensor_x, sensor_y, sensor_z),
		octomap::point3d(end_x, end_y, end_z), insertionOptions.maxrange,
		insertionOptions.pruning);
}
void CColouredOctoMap::updateVoxel(
	const double x, const double y, const double z, bool occupied)
{
	m_impl->m_octomap.updateNode(x, y, z, occupied);
}
bool CColouredOctoMap::isPointWithinOctoMap(
	const float x, const float y, const float z) const
{
	octomap::OcTreeKey key;
	return m_impl->m_octomap.coordToKeyChecked(octomap::point3d(x, y, z), key);
}

double CColouredOctoMap::getResolution() const
{
	return m_impl->m_octomap.getResolution();
}
unsigned int CColouredOctoMap::getTreeDepth() const
{
	return m_impl->m_octomap.getTreeDepth();
}
size_t CColouredOctoMap::size() const { return m_impl->m_octomap.size(); }
size_t CColouredOctoMap::memoryUsage() const
{
	return m_impl->m_octomap.memoryUsage();
}
size_t CColouredOctoMap::memoryUsageNode() const
{
	return m_impl->m_octomap.memoryUsageNode();
}
size_t CColouredOctoMap::memoryFullGrid() const
{
	return m_impl->m_octomap.memoryFullGrid();
}
double CColouredOctoMap::volume() { return m_impl->m_octomap.volume(); }
void CColouredOctoMap::getMetricSize(double& x, double& y, double& z)
{
	return m_impl->m_octomap.getMetricSize(x, y, z);
}
void CColouredOctoMap::getMetricSize(double& x, double& y, double& z) const
{
	return m_impl->m_octomap.getMetricSize(x, y, z);
}
void CColouredOctoMap::getMetricMin(double& x, double& y, double& z)
{
	return m_impl->m_octomap.getMetricMin(x, y, z);
}
void CColouredOctoMap::getMetricMin(double& x, double& y, double& z) const
{
	return m_impl->m_octomap.getMetricMin(x, y, z);
}
void CColouredOctoMap::getMetricMax(double& x, double& y, double& z)
{
	return m_impl->m_octomap.getMetricMax(x, y, z);
}
void CColouredOctoMap::getMetricMax(double& x, double& y, double& z) const
{
	return m_impl->m_octomap.getMetricMax(x, y, z);
}
size_t CColouredOctoMap::calcNumNodes() const
{
	return m_impl->m_octomap.calcNumNodes();
}
size_t CColouredOctoMap::getNumLeafNodes() const
{
	return m_impl->m_octomap.getNumLeafNodes();
}
void CColouredOctoMap::setOccupancyThres(double prob)
{
	m_impl->m_octomap.setOccupancyThres(prob);
}
void CColouredOctoMap::setProbHit(double prob)
{
	m_impl->m_octomap.setProbHit(prob);
}
void CColouredOctoMap::setProbMiss(double prob)
{
	m_impl->m_octomap.setProbMiss(prob);
}
void CColouredOctoMap::setClampingThresMin(double thresProb)
{
	m_impl->m_octomap.setClampingThresMin(thresProb);
}
void CColouredOctoMap::setClampingThresMax(double thresProb)
{
	m_impl->m_octomap.setClampingThresMax(thresProb);
}
double CColouredOctoMap::getOccupancyThres() const
{
	return m_impl->m_octomap.getOccupancyThres();
}
float CColouredOctoMap::getOccupancyThresLog() const
{
	return m_impl->m_octomap.getOccupancyThresLog();
}
double CColouredOctoMap::getProbHit() const
{
	return m_impl->m_octomap.getProbHit();
}
float CColouredOctoMap::getProbHitLog() const
{
	return m_impl->m_octomap.getProbHitLog();
}
double CColouredOctoMap::getProbMiss() const
{
	return m_impl->m_octomap.getProbMiss();
}
float CColouredOctoMap::getProbMissLog() const
{
	return m_impl->m_octomap.getProbMissLog();
}
double CColouredOctoMap::getClampingThresMin() const
{
	return m_impl->m_octomap.getClampingThresMin();
}
float CColouredOctoMap::getClampingThresMinLog() const
{
	return m_impl->m_octomap.getClampingThresMinLog();
}
double CColouredOctoMap::getClampingThresMax() const
{
	return m_impl->m_octomap.getClampingThresMax();
}
float CColouredOctoMap::getClampingThresMaxLog() const
{
	return m_impl->m_octomap.getClampingThresMaxLog();
}
void CColouredOctoMap::internal_clear() { m_impl->m_octomap.clear(); }
