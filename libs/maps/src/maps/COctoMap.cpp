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

/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
COctoMap::COctoMap(const double resolution) :
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
		MRPT_TODO("write");
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
			MRPT_TODO("read")
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


	if ( CLASS_ID(CObservation2DRangeScan)==obs->GetRuntimeClass())
	{
	   /********************************************************************
				OBSERVATION TYPE: CObservation2DRangeScan
		********************************************************************/
		const CObservation2DRangeScan	*o = static_cast<const CObservation2DRangeScan*>( obs );

		// Compose the actual 3D pose of the sensor:
		CPose3D sensorPose3D(UNINITIALIZED_POSE);
		sensorPose3D.composeFrom(robotPose3D, o->sensorPose);

		octomap::point3d sensorPt(sensorPose3D.x(),sensorPose3D.y(),sensorPose3D.z());

		// Build a 2D points-map representation of the points from the scan:
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
			sensorPose3D.composePoint(pt.x,pt.y,pt.z,  gx,gy,gz);

			// Add to this map:
			scan.push_back(gx,gy,gz);
		}

		// Insert:
		OCTOMAP_PTR->insertScan(scan, sensorPt, insertionOptions.maxrange, insertionOptions.pruning);

		return true;
	}
	// else ...


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
COctoMap::TInsertionOptions::TInsertionOptions() :
	maxrange (-1.),
	pruning  (true)
{
}

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
	MRPT_TODO("x")
	return 0;
}

/** Computes the matchings between this and another 2D points map.
	This includes finding:
		- The set of points pairs in each map
		- The mean squared distance between corresponding pairs.
	This method is the most time critical one into the ICP algorithm.

	* \param  otherMap					  [IN] The other map to compute the matching with.
	* \param  otherMapPose				  [IN] The pose of the other map as seen from "this".
	* \param  maxDistForCorrespondence   [IN] Maximum 2D linear distance between two points to be matched.
	* \param  maxAngularDistForCorrespondence [IN] In radians: The aim is to allow larger distances to more distant correspondences.
	* \param  angularDistPivotPoint      [IN] The point used to calculate distances from in both maps.
	* \param  correspondences			  [OUT] The detected matchings pairs.
	* \param  correspondencesRatio		  [OUT] The ratio [0,1] of points in otherMap with at least one correspondence.
	* \param  sumSqrDist				  [OUT] The sum of all matched points squared distances.If undesired, set to NULL, as default.
	* \param  onlyKeepTheClosest         [IN] If set to true, only the closest correspondence will be returned. If false (default) all are returned.
	*
	* \sa compute3DMatchingRatio
	*/
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
	MRPT_TODO("x")
}

/** Computes the matchings between this and another 3D points map - method used in 3D-ICP.
	This method finds the set of point pairs in each map.

	The method is the most time critical one into the ICP algorithm.

	* \param  otherMap					  [IN] The other map to compute the matching with.
	* \param  otherMapPose				  [IN] The pose of the other map as seen from "this".
	* \param  maxDistForCorrespondence   [IN] Maximum 2D linear distance between two points to be matched.
	* \param  maxAngularDistForCorrespondence [IN] In radians: The aim is to allow larger distances to more distant correspondences.
	* \param  angularDistPivotPoint      [IN] The point used to calculate distances from in both maps.
	* \param  correspondences			  [OUT] The detected matchings pairs.
	* \param  correspondencesRatio		  [OUT] The ratio [0,1] of points in otherMap with at least one correspondence.
	* \param  sumSqrDist				  [OUT] The sum of all matched points squared distances.If undesired, set to NULL, as default.
	* \param  onlyKeepTheClosest         [IN] If set to true, only the closest correspondence will be returned. If false (default) all are returned.
	*
	* \sa compute3DMatchingRatio
	*/
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
	MRPT_TODO("x")
}


/** Computes the ratio in [0,1] of correspondences between "this" and the "otherMap" map, whose 6D pose relative to "this" is "otherMapPose"
	*   In the case of a multi-metric map, this returns the average between the maps. This method always return 0 for grid maps.
	* \param  otherMap					  [IN] The other map to compute the matching with.
	* \param  otherMapPose				  [IN] The 6D pose of the other map as seen from "this".
	* \param  minDistForCorr			  [IN] The minimum distance between 2 non-probabilistic map elements for counting them as a correspondence.
	* \param  minMahaDistForCorr		  [IN] The minimum Mahalanobis distance between 2 probabilistic map elements for counting them as a correspondence.
	*
	* \return The matching ratio [0,1]
	* \sa computeMatchingWith2D
	*/
float COctoMap::compute3DMatchingRatio(
	const CMetricMap								*otherMap,
	const CPose3D							&otherMapPose,
	float									minDistForCorr ,
	float									minMahaDistForCorr) const
{
	MRPT_TODO("x")
	return 0;
}

/** This virtual method saves the map to a file "filNamePrefix"+< some_file_extension >, as an image or in any other applicable way (Notice that other methods to save the map may be implemented in classes implementing this virtual interface).
	*/
void COctoMap::saveMetricMapRepresentationToFile(const std::string	&filNamePrefix) const
{
	MRPT_TODO("x")
}

/** Returns a 3D object representing the map.
	*/
void COctoMap::getAs3DObject( mrpt::opengl::CSetOfObjectsPtr &outObj ) const
{
	MRPT_TODO("x")
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
