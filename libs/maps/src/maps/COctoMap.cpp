/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */

#include <mrpt/maps.h>  // Precompiled header

#include <mrpt/slam/COctoMap.h>
#include <mrpt/slam/CPointsMap.h>
#include <mrpt/slam/CObservation2DRangeScan.h>
#include <mrpt/slam/CObservation3DRangeScan.h>

#include <mrpt/opengl/COctoMapVoxels.h>
#include <mrpt/opengl/COpenGLScene.h>

#include <mrpt/system/filesystem.h>
#include <mrpt/utils/CMemoryChunk.h>
#include <mrpt/utils/CFileOutputStream.h>

#include <octomap/octomap.h>


using namespace std;
using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::math;

IMPLEMENTS_SERIALIZABLE(COctoMap, CMetricMap,mrpt::slam)

#define OCTOMAP_PTR        static_cast<octomap::OcTree*>(m_octomap)
#define OCTOMAP_PTR_CONST  static_cast<const octomap::OcTree*>(m_octomap)

#define PARENT_OCTOMAP_PTR        static_cast<octomap::OcTree*>(m_parent->m_octomap)
#define PARENT_OCTOMAP_PTR_CONST  static_cast<const octomap::OcTree*>(m_parent->m_octomap)

/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
COctoMap::COctoMap(const double resolution) :
	insertionOptions(*this),
	m_octomap(NULL)
{
	this->allocOctomap(resolution);
}

/*---------------------------------------------------------------
						Destructor
  ---------------------------------------------------------------*/
COctoMap::~COctoMap()
{
	this->freeOctomap();
}

void COctoMap::freeOctomap()
{
	if (m_octomap)
	{
		delete OCTOMAP_PTR;
		m_octomap = NULL;
	}
}

void COctoMap::allocOctomap(double resolution)
{
	freeOctomap();
	m_octomap = static_cast<void*>(new octomap::OcTree(resolution));
}


/*---------------------------------------------------------------
					writeToStream
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void  COctoMap::writeToStream(CStream &out, int *version) const
{
	if (version)
		*version = 0;
	else
	{
		this->likelihoodOptions.writeToStream(out);
		
		CMemoryChunk chunk;
		const string	tmpFil = mrpt::system::getTempFileName();
		OCTOMAP_PTR->writeBinary(tmpFil);
		chunk.loadBufferFromFile(tmpFil);
		mrpt::system::deleteFile(tmpFil);

		out << chunk;
	}
}

/*---------------------------------------------------------------
					readFromStream
   Implements the reading from a CStream capability of
      CSerializable objects
  ---------------------------------------------------------------*/
void  COctoMap::readFromStream(CStream &in, int version)
{
	switch(version)
	{
	case 0:
		{
			this->likelihoodOptions.readFromStream(in);

			this->clear();
			
			CMemoryChunk chunk;
			in >> chunk;

			if (chunk.getTotalBytesCount())
			{
				const string	tmpFil = mrpt::system::getTempFileName();
				if (!chunk.saveBufferToFile( tmpFil ) ) THROW_EXCEPTION("Error saving temporary file");
				OCTOMAP_PTR->readBinary(tmpFil);
				mrpt::system::deleteFile( tmpFil );
			}

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};

}

/*---------------------------------------------------------------
					Clear
  ---------------------------------------------------------------*/
void  COctoMap::internal_clear()
{
	OCTOMAP_PTR->clear();
}

/*---------------------------------------------------------------
				insertObservation
 ---------------------------------------------------------------*/
bool COctoMap::internal_insertObservation(const CObservation *obs,const CPose3D *robotPose)
{
	CPose3D		robotPose3D;
	if (robotPose) // Default values are (0,0,0)
		robotPose3D = (*robotPose);


	if ( IS_CLASS(obs,CObservation2DRangeScan) )
	{
	   /********************************************************************
				OBSERVATION TYPE: CObservation2DRangeScan
		********************************************************************/
		const CObservation2DRangeScan	*o = static_cast<const CObservation2DRangeScan*>( obs );

		// Build a points-map representation of the points from the scan (coordinates are wrt the robot base)
		octomap::point3d sensorPt(robotPose3D.x(),robotPose3D.y(),robotPose3D.z());
		const CPointsMap *scanPts = o->buildAuxPointsMap<mrpt::slam::CPointsMap>();
		const size_t nPts = scanPts->size();

		// Transform 3D point cloud:
		octomap::Pointcloud  scan;
		scan.reserve(nPts);

		mrpt::math::TPoint3Df pt;
		for (size_t i=0;i<nPts;i++)
		{
			// Load the next point:
			scanPts->getPointFast(i,pt.x,pt.y,pt.z);

			// Translation:
			double gx,gy,gz;
			robotPose3D.composePoint(pt.x,pt.y,pt.z,  gx,gy,gz);

			// Add to this map:
			scan.push_back(gx,gy,gz);
		}

		// Insert:
		OCTOMAP_PTR->insertScan(scan, sensorPt, insertionOptions.maxrange, insertionOptions.pruning);

		return true;
	}
	else if ( IS_CLASS(obs,CObservation3DRangeScan) )
	{
	   /********************************************************************
				OBSERVATION TYPE: CObservation3DRangeScan
		********************************************************************/
		const CObservation3DRangeScan	*o = static_cast<const CObservation3DRangeScan*>( obs );

		// Build a points-map representation of the points from the scan (coordinates are wrt the robot base)
		octomap::point3d sensorPt(robotPose3D.x(),robotPose3D.y(),robotPose3D.z());

		if (!o->hasPoints3D)
			return false; 

		o->load(); // Just to make sure the points are loaded from an external source, if that's the case...
		const size_t sizeRangeScan = o->points3D_x.size();

		// Transform 3D point cloud:
		octomap::Pointcloud  scan;
		scan.reserve(sizeRangeScan);

		// For quicker access to values as "float" instead of "doubles":
		mrpt::math::CMatrixDouble44  H;
		robotPose3D.getHomogeneousMatrix(H);
		const float	m00 = H.get_unsafe(0,0);
		const float	m01 = H.get_unsafe(0,1);
		const float	m02 = H.get_unsafe(0,2);
		const float	m03 = H.get_unsafe(0,3);
		const float	m10 = H.get_unsafe(1,0);
		const float	m11 = H.get_unsafe(1,1);
		const float	m12 = H.get_unsafe(1,2);
		const float	m13 = H.get_unsafe(1,3);
		const float	m20 = H.get_unsafe(2,0);
		const float	m21 = H.get_unsafe(2,1);
		const float	m22 = H.get_unsafe(2,2);
		const float	m23 = H.get_unsafe(2,3);

		mrpt::math::TPoint3Df pt;
		for (size_t i=0;i<sizeRangeScan;i++)
		{
			pt.x = o->points3D_x[i]; 
			pt.y = o->points3D_y[i];
			pt.z = o->points3D_z[i];
			
			// Valid point?
			if ( pt.x!=0 || pt.y!=0 || pt.z!=0 )
			{
				// Translation:
				const float gx = m00*pt.x + m01*pt.y + m02*pt.z + m03;
				const float gy = m10*pt.x + m11*pt.y + m12*pt.z + m13;
				const float gz = m20*pt.x + m21*pt.y + m22*pt.z + m23;

				// Add to this map:
				scan.push_back(gx,gy,gz);
			}
		}

		// Insert:
		OCTOMAP_PTR->insertScan(scan, sensorPt, insertionOptions.maxrange, insertionOptions.pruning);

		return true;
	}

	return false;
}


/*---------------------------------------------------------------
				isEmpty
 ---------------------------------------------------------------*/
bool  COctoMap::isEmpty() const
{
	return OCTOMAP_PTR->size()==1;
}

/*---------------------------------------------------------------
				TInsertionOptions
 ---------------------------------------------------------------*/
COctoMap::TInsertionOptions::TInsertionOptions(COctoMap &parent) :
	maxrange (-1.),
	pruning  (true),
	m_parent (&parent),
	// Default values from octomap:
	occupancyThres (0.5),
	probHit(0.7),
	probMiss(0.4),
	clampingThresMin(0.1192),
	clampingThresMax(0.971)
{
}

COctoMap::TInsertionOptions::TInsertionOptions() :
	maxrange (-1.),
	pruning  (true),
	m_parent (NULL),
	// Default values from octomap:
	occupancyThres (0.5),
	probHit(0.7),
	probMiss(0.4),
	clampingThresMin(0.1192),
	clampingThresMax(0.971)
{
}

COctoMap::TInsertionOptions & COctoMap::TInsertionOptions::operator = (const COctoMap::TInsertionOptions &o)
{
	// Copy all but the m_parent pointer!
	maxrange = o.maxrange;
	pruning  = o.pruning;

	const bool o_has_parent = o.m_parent.get()!=NULL;

	setOccupancyThres( o_has_parent ? o.getOccupancyThres() : o.occupancyThres );
	setProbHit( o_has_parent ? o.getProbHit() : o.probHit );
	setProbMiss( o_has_parent ? o.getProbMiss() : o.probMiss );
	setClampingThresMin( o_has_parent ? o.getClampingThresMin() : o.clampingThresMin );
	setClampingThresMax( o_has_parent ? o.getClampingThresMax() : o.clampingThresMax );

	return *this;
}


void COctoMap::TInsertionOptions::setOccupancyThres(double prob) { if(m_parent.get()) PARENT_OCTOMAP_PTR->setOccupancyThres(prob); }
void COctoMap::TInsertionOptions::setProbHit(double prob) { if(m_parent.get()) PARENT_OCTOMAP_PTR->setProbHit(prob); }
void COctoMap::TInsertionOptions::setProbMiss(double prob) { if(m_parent.get()) PARENT_OCTOMAP_PTR->setProbMiss(prob); }
void COctoMap::TInsertionOptions::setClampingThresMin(double thresProb) { if(m_parent.get()) PARENT_OCTOMAP_PTR->setClampingThresMin(thresProb); }
void COctoMap::TInsertionOptions::setClampingThresMax(double thresProb) { if(m_parent.get()) PARENT_OCTOMAP_PTR->setClampingThresMax(thresProb); }

double COctoMap::TInsertionOptions::getOccupancyThres() const { if(m_parent.get()) return PARENT_OCTOMAP_PTR_CONST->getOccupancyThres(); else return this->occupancyThres; }
float COctoMap::TInsertionOptions::getOccupancyThresLog() const  { return PARENT_OCTOMAP_PTR_CONST->getOccupancyThresLog() ; }

double COctoMap::TInsertionOptions::getProbHit() const  { if(m_parent.get()) return PARENT_OCTOMAP_PTR_CONST->getProbHit(); else return this->probHit; }
float COctoMap::TInsertionOptions::getProbHitLog() const { return PARENT_OCTOMAP_PTR_CONST->getProbHitLog(); }
double COctoMap::TInsertionOptions::getProbMiss() const { if(m_parent.get()) return PARENT_OCTOMAP_PTR_CONST->getProbMiss(); else return this->probMiss; }
float COctoMap::TInsertionOptions::getProbMissLog() const { return PARENT_OCTOMAP_PTR_CONST->getProbMissLog(); }

double COctoMap::TInsertionOptions::getClampingThresMin() const { if(m_parent.get()) return PARENT_OCTOMAP_PTR_CONST->getClampingThresMin(); else return this->clampingThresMin; }
float COctoMap::TInsertionOptions::getClampingThresMinLog() const { return PARENT_OCTOMAP_PTR_CONST->getClampingThresMinLog(); }
double COctoMap::TInsertionOptions::getClampingThresMax() const { if(m_parent.get()) return PARENT_OCTOMAP_PTR_CONST->getClampingThresMax(); else return this->clampingThresMax; }
float COctoMap::TInsertionOptions::getClampingThresMaxLog() const { return PARENT_OCTOMAP_PTR_CONST->getClampingThresMaxLog(); }

COctoMap::TLikelihoodOptions::TLikelihoodOptions() :
	sigma_dist			( 0.05 ),
	max_corr_distance	( 1.0 ),
	decimation			( 1 )
{

}

void COctoMap::TLikelihoodOptions::writeToStream(CStream &out) const
{
	const int8_t version = 0;
	out << version;
	out << sigma_dist << max_corr_distance << decimation;
}

void COctoMap::TLikelihoodOptions::readFromStream(CStream &in)
{
	int8_t version;
	in >> version;
	switch(version)
	{
		case 0:
		{
			in >> sigma_dist >> max_corr_distance >> decimation;
		}
		break;
		default: MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	}
}


/*---------------------------------------------------------------
					dumpToTextStream
  ---------------------------------------------------------------*/
void  COctoMap::TInsertionOptions::dumpToTextStream(CStream	&out) const
{
	out.printf("\n----------- [COctoMap::TInsertionOptions] ------------ \n\n");

	LOADABLEOPTS_DUMP_VAR(maxrange,double);
	LOADABLEOPTS_DUMP_VAR(pruning,bool);

	LOADABLEOPTS_DUMP_VAR(getOccupancyThres(),double);
	LOADABLEOPTS_DUMP_VAR(getProbHit(),double);
	LOADABLEOPTS_DUMP_VAR(getProbMiss(),double);
	LOADABLEOPTS_DUMP_VAR(getClampingThresMin(),double);
	LOADABLEOPTS_DUMP_VAR(getClampingThresMax(),double);

	out.printf("\n");
}

void  COctoMap::TLikelihoodOptions::dumpToTextStream(CStream	&out) const
{
	out.printf("\n----------- [COctoMap::TLikelihoodOptions] ------------ \n\n");

	LOADABLEOPTS_DUMP_VAR(sigma_dist,double);
	LOADABLEOPTS_DUMP_VAR(max_corr_distance,double);
	LOADABLEOPTS_DUMP_VAR(decimation,int);
}

/*---------------------------------------------------------------
					loadFromConfigFile
  ---------------------------------------------------------------*/
void  COctoMap::TInsertionOptions::loadFromConfigFile(
	const mrpt::utils::CConfigFileBase  &iniFile,
	const string &section)
{
	MRPT_LOAD_CONFIG_VAR(maxrange,double, iniFile,section);
	MRPT_LOAD_CONFIG_VAR(pruning,bool, iniFile,section);

	MRPT_LOAD_CONFIG_VAR(occupancyThres,double, iniFile,section);
	MRPT_LOAD_CONFIG_VAR(probHit,double, iniFile,section);
	MRPT_LOAD_CONFIG_VAR(probMiss,double, iniFile,section);
	MRPT_LOAD_CONFIG_VAR(clampingThresMin,double, iniFile,section);
	MRPT_LOAD_CONFIG_VAR(clampingThresMax,double, iniFile,section);

	// Set loaded options into the actual octomap object, if any:
	this->setOccupancyThres(occupancyThres);
	this->setProbHit(probHit);
	this->setProbMiss(probMiss);
	this->setClampingThresMin(clampingThresMin);
	this->setClampingThresMax(clampingThresMax);
}

void  COctoMap::TLikelihoodOptions::loadFromConfigFile(
	const mrpt::utils::CConfigFileBase  &iniFile,
	const string &section)
{
	MRPT_LOAD_CONFIG_VAR(sigma_dist,double,iniFile,section);
	MRPT_LOAD_CONFIG_VAR(max_corr_distance,double,iniFile,section);
	MRPT_LOAD_CONFIG_VAR(decimation,int,iniFile,section);
}


/*---------------------------------------------------------------
					computeObservationLikelihood
  ---------------------------------------------------------------*/
double COctoMap::computeObservationLikelihood( const CObservation *obs, const CPose3D &takenFrom )
{
	MRPT_TODO("Implement computeObservationLikelihood()")
	THROW_EXCEPTION("TODO");
	return 0;
}

void COctoMap::computeMatchingWith2D(
	const CMetricMap						*otherMap,
	const CPose2D							&otherMapPose,
	float									maxDistForCorrespondence,
	float									maxAngularDistForCorrespondence,
	const CPose2D							&angularDistPivotPoint,
	TMatchingPairList						&correspondences,
	float									&correspondencesRatio,
	float									*sumSqrDist	,
	bool									onlyKeepTheClosest,
	bool									onlyUniqueRobust,
	const size_t                            decimation_other_map_points,
	const size_t                            offset_other_map_points ) const
{
	THROW_EXCEPTION("Method not implemented in this class of map")
}

void COctoMap::computeMatchingWith3D(
	const CMetricMap						*otherMap,
	const CPose3D							&otherMapPose,
	float									maxDistForCorrespondence,
	float									maxAngularDistForCorrespondence,
	const CPoint3D							&angularDistPivotPoint,
	TMatchingPairList						&correspondences,
	float									&correspondencesRatio,
	float									*sumSqrDist,
	bool									onlyKeepTheClosest,
	bool									onlyUniqueRobust ,
	const size_t                            decimation_other_map_points ,
	const size_t                            offset_other_map_points ) const
{
	THROW_EXCEPTION("Method not implemented in this class of map")
}

float COctoMap::compute3DMatchingRatio(
	const CMetricMap								*otherMap,
	const CPose3D							&otherMapPose,
	float									minDistForCorr ,
	float									minMahaDistForCorr) const
{
	THROW_EXCEPTION("Method not implemented in this class of map")
}

/** This virtual method saves the map to a file "filNamePrefix"+< some_file_extension >, as an image or in any other applicable way (Notice that other methods to save the map may be implemented in classes implementing this virtual interface).
	*/
void COctoMap::saveMetricMapRepresentationToFile(const std::string	&filNamePrefix) const
{
	MRPT_START

	// Save as 3D Scene:
	{
		mrpt::opengl::COpenGLScene scene;
		mrpt::opengl::CSetOfObjectsPtr obj3D = mrpt::opengl::CSetOfObjects::Create();

		this->getAs3DObject(obj3D);

		scene.insert(obj3D);

		const string fil = filNamePrefix + string("_3D.3Dscene");
		mrpt::utils::CFileOutputStream	f(fil);
		f << scene;
	}

	// Save as ".bt" file (a binary format from the octomap lib):
	{
		const string fil = filNamePrefix + string("_binary.bt");
		OCTOMAP_PTR->writeBinary(fil);
	}

	MRPT_END
}

double COctoMap::getResolution() const {
	return OCTOMAP_PTR_CONST->getResolution();
}
unsigned int COctoMap::getTreeDepth() const {
	return OCTOMAP_PTR_CONST->getTreeDepth();
}
size_t COctoMap::size() const {
	return  OCTOMAP_PTR_CONST->size();
}
size_t COctoMap::memoryUsage() const {
	return  OCTOMAP_PTR_CONST->memoryUsage();
}
size_t COctoMap::memoryUsageNode() const {
	return  OCTOMAP_PTR_CONST->memoryUsageNode();
}
size_t COctoMap::memoryFullGrid() const {
	return  OCTOMAP_PTR_CONST->memoryFullGrid();
}
double COctoMap::volume() const {
	return OCTOMAP_PTR_CONST->volume();
}
void COctoMap::getMetricSize(double& x, double& y, double& z) {
	return  OCTOMAP_PTR_CONST->getMetricSize(x,y,z);
}
void COctoMap::getMetricSize(double& x, double& y, double& z) const {
	return  OCTOMAP_PTR_CONST->getMetricSize(x,y,z);
}
void COctoMap::getMetricMin(double& x, double& y, double& z) {
	return  OCTOMAP_PTR_CONST->getMetricMin(x,y,z);
}
void COctoMap::getMetricMin(double& x, double& y, double& z) const {
	return  OCTOMAP_PTR_CONST->getMetricMin(x,y,z);
}
void COctoMap::getMetricMax(double& x, double& y, double& z) {
	return  OCTOMAP_PTR_CONST->getMetricMax(x,y,z);
}
void COctoMap::getMetricMax(double& x, double& y, double& z) const {
	return  OCTOMAP_PTR_CONST->getMetricMax(x,y,z);
}
size_t COctoMap::calcNumNodes() const {
	return  OCTOMAP_PTR_CONST->calcNumNodes();
}
size_t COctoMap::getNumLeafNodes() const {
	return  OCTOMAP_PTR_CONST->getNumLeafNodes();
}

/** Check whether the given point lies within the volume covered by the octomap (that is, whether it is "mapped") */
bool COctoMap::isPointWithinOctoMap(const float x,const float y,const float z) const
{
    octomap::OcTreeKey key;
	return OCTOMAP_PTR_CONST->coordToKeyChecked(octomap::point3d(x,y,z), key);
}

/** Get the occupancy probability [0,1] of a point 
* \return false if the point is not mapped, in which case the returned "prob" is undefined. */
bool COctoMap::getPointOccupancy(const float x,const float y,const float z, double &prob_occupancy) const
{
    octomap::OcTreeKey key;
	if (OCTOMAP_PTR_CONST->coordToKeyChecked(octomap::point3d(x,y,z), key))
	{
		octomap::OcTreeNode *node = OCTOMAP_PTR_CONST->search(key,0 /*depth*/);
		if (!node) return false;

		prob_occupancy = node->getOccupancy();
		return true;
	}
	else return false;
}

/** Manually updates the occupancy of the voxel at (x,y,z) as being occupied (true) or free (false), using the log-odds parameters in \a insertionOptions */
void COctoMap::updateVoxel(const double x, const double y, const double z, bool occupied)
{
	OCTOMAP_PTR->updateNode(x,y,z, occupied);
}

void COctoMap::insertPointCloud(const CPointsMap &ptMap, const float sensor_x,const float sensor_y,const float sensor_z)
{
	MRPT_START

	const octomap::point3d sensorPt(sensor_x,sensor_y,sensor_z);
	
	size_t N;
	const float *xs,*ys,*zs;
	ptMap.getPointsBuffer(N,xs,ys,zs);

	for (size_t i=0;i<N;i++)
		OCTOMAP_PTR->insertRay(sensorPt, octomap::point3d(xs[i],ys[i],zs[i]), insertionOptions.maxrange,insertionOptions.pruning);

	MRPT_END
}

/** Just like insertPointCloud but with a single ray. */
void COctoMap::insertRay(const float end_x,const float end_y,const float end_z,const float sensor_x,const float sensor_y,const float sensor_z)
{
	OCTOMAP_PTR->insertRay( octomap::point3d(sensor_x,sensor_y,sensor_z), octomap::point3d(end_x,end_y,end_z), insertionOptions.maxrange,insertionOptions.pruning);
}
