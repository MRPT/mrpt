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
#ifndef MRPT_COctoMap_H
#define MRPT_COctoMap_H

#include <mrpt/slam/CMetricMap.h>
#include <mrpt/utils/CLoadableOptions.h>

#include <mrpt/maps/link_pragmas.h>

namespace mrpt
{
	namespace slam
	{
		class CPointsMap;
		class CObservation2DRangeScan;
		class CObservation3DRangeScan;

		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( COctoMap , CMetricMap, MAPS_IMPEXP )

		/** A three-dimensional probabilistic occupancy grid, implemented as an octo-tree with the "octomap" C++ library.
		 *  This class represents a 3D map where each voxel only contains an "occupancy" property.
		 *
		 * As with any other mrpt::slam::CMetricMap, you can obtain a 3D representation of the map calling getAs3DObject()
		 *
		 * \sa CMetricMap
	  	 * \ingroup mrpt_maps_grp
		 */
		class MAPS_IMPEXP COctoMap : public CMetricMap
		{
			// This must be added to any CSerializable derived class:
			DEFINE_SERIALIZABLE( COctoMap )

		 public:
			 COctoMap(const double resolution=0.10);          //!< Default constructor
			 virtual ~COctoMap(); //!< Destructor

		 /** With this struct options are provided to the observation insertion process.
		  * \sa CObservation::insertObservationInto()
		  */
		 struct MAPS_IMPEXP TInsertionOptions : public utils::CLoadableOptions
		 {
			/** Initilization of default parameters */
			TInsertionOptions( );
			/** See utils::CLoadableOptions */
			void  loadFromConfigFile(const mrpt::utils::CConfigFileBase  &source,const std::string &section);
			/** See utils::CLoadableOptions */
			void  dumpToTextStream(CStream	&out) const;

			double maxrange;  //!< maximum range for how long individual beams are inserted (default -1: complete beam)
			bool pruning;     //!< whether the tree is (losslessly) pruned after insertion (default: true)
		 };

		TInsertionOptions insertionOptions; //!< The options used when inserting observations in the map

		 /** Options used when evaluating "computeObservationLikelihood"
		  * \sa CObservation::computeObservationLikelihood
		  */
		 struct MAPS_IMPEXP TLikelihoodOptions: public utils::CLoadableOptions
		 {
			/** Initilization of default parameters
			 */
			TLikelihoodOptions( );
			virtual ~TLikelihoodOptions() {}

			/** See utils::CLoadableOptions */
			void  loadFromConfigFile(
				const mrpt::utils::CConfigFileBase  &source,
				const std::string &section);

			/** See utils::CLoadableOptions */
			void  dumpToTextStream(CStream	&out) const;

			void writeToStream(CStream &out) const;		//!< Binary dump to stream
			void readFromStream(CStream &in);			//!< Binary dump to stream

			double 		sigma_dist; //!< Sigma (standard deviation, in meters) of the exponential used to model the likelihood (default= 0.5meters)
			double 		max_corr_distance; //!< Maximum distance in meters to consider for the numerator divided by "sigma_dist", so that each point has a minimum (but very small) likelihood to avoid underflows (default=1.0 meters)
			uint32_t	decimation; //!< Speed up the likelihood computation by considering only one out of N rays (default=1)
		 };

		 TLikelihoodOptions  likelihoodOptions;

		/** Returns true if the map is empty/no observation has been inserted.
			*/
		virtual bool  isEmpty() const;


		/** Computes the log-likelihood of a given observation given an arbitrary robot 3D pose.
			*
			* \param takenFrom The robot's pose the observation is supposed to be taken from.
			* \param obs The observation.
			* \return This method returns a log-likelihood.
			*
			* \sa Used in particle filter algorithms, see: CMultiMetricMapPDF::update
			*/
		virtual double	 computeObservationLikelihood( const CObservation *obs, const CPose3D &takenFrom );

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
		virtual void  computeMatchingWith2D(
			const CMetricMap						*otherMap,
			const CPose2D							&otherMapPose,
			float									maxDistForCorrespondence,
			float									maxAngularDistForCorrespondence,
			const CPose2D							&angularDistPivotPoint,
			TMatchingPairList						&correspondences,
			float									&correspondencesRatio,
			float									*sumSqrDist	= NULL,
			bool									onlyKeepTheClosest = true,
			bool									onlyUniqueRobust = false,
			const size_t                            decimation_other_map_points = 1,
			const size_t                            offset_other_map_points = 0 ) const;

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
		virtual void  computeMatchingWith3D(
			const CMetricMap						*otherMap,
			const CPose3D							&otherMapPose,
			float									maxDistForCorrespondence,
			float									maxAngularDistForCorrespondence,
			const CPoint3D							&angularDistPivotPoint,
			TMatchingPairList						&correspondences,
			float									&correspondencesRatio,
			float									*sumSqrDist	= NULL,
			bool									onlyKeepTheClosest = true,
			bool									onlyUniqueRobust = false,
			const size_t                            decimation_other_map_points = 1,
			const size_t                            offset_other_map_points = 0 ) const;


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
		virtual float  compute3DMatchingRatio(
			const CMetricMap								*otherMap,
			const CPose3D							&otherMapPose,
			float									minDistForCorr = 0.10f,
			float									minMahaDistForCorr = 2.0f
		) const;

		/** This virtual method saves the map to a file "filNamePrefix"+< some_file_extension >, as an image or in any other applicable way (Notice that other methods to save the map may be implemented in classes implementing this virtual interface).
			*/
		virtual void  saveMetricMapRepresentationToFile(
			const std::string	&filNamePrefix
		) const;

		/** Returns a 3D object representing the map.
			*/
		virtual void  getAs3DObject( mrpt::opengl::CSetOfObjectsPtr	&outObj ) const;

		/** @name Direct access to octomap library methods
		    @{ */

		double getResolution() const;
		unsigned int getTreeDepth () const;
		/// \return The number of nodes in the tree
		size_t size() const;
		/// \return Memory usage of the complete octree in bytes (may vary between architectures)
		size_t memoryUsage() const;
		/// \return Memory usage of the a single octree node
		size_t memoryUsageNode() const;
		/// \return Memory usage of a full grid of the same size as the OcTree in bytes (for comparison)
		size_t memoryFullGrid() const;
		double volume() const;
		/// Size of OcTree (all known space) in meters for x, y and z dimension
		void getMetricSize(double& x, double& y, double& z);
		/// Size of OcTree (all known space) in meters for x, y and z dimension
		void getMetricSize(double& x, double& y, double& z) const;
		/// minimum value of the bounding box of all known space in x, y, z
		void getMetricMin(double& x, double& y, double& z);
		/// minimum value of the bounding box of all known space in x, y, z
		void getMetricMin(double& x, double& y, double& z) const;
		/// maximum value of the bounding box of all known space in x, y, z
		void getMetricMax(double& x, double& y, double& z);
		/// maximum value of the bounding box of all known space in x, y, z
		void getMetricMax(double& x, double& y, double& z) const;

		/// Traverses the tree to calculate the total number of nodes
		size_t calcNumNodes() const;

		/// Traverses the tree to calculate the total number of leaf nodes
		size_t getNumLeafNodes() const;

		/** @} */


	protected:
		/** Clear the map, erasing all contents.
			*/
		virtual void  internal_clear();

		/** Internal method called by insertObservation() */
		virtual bool  internal_insertObservation(
			const CObservation *obs,
			const CPose3D *robotPose = NULL );

		void freeOctomap();  //!< Internal use only
		void allocOctomap(double resolution); //!< Internal use only

		void *m_octomap; //!< The (user-opaque) pointer to the actual octo-map object.

		}; // End of class def.
	} // End of namespace

} // End of namespace

#endif
