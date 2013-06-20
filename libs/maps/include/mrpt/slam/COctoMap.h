/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
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
#include <mrpt/utils/safe_pointers.h>
#include <mrpt/otherlibs/octomap/octomap.h>

#include <mrpt/maps/link_pragmas.h>

namespace mrpt
{
	namespace opengl { class COctoMapVoxels; }
	namespace slam
	{
		class CPointsMap;
		class CObservation2DRangeScan;
		class CObservation3DRangeScan;

		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( COctoMap , CMetricMap, MAPS_IMPEXP )

		/** A three-dimensional probabilistic occupancy grid, implemented as an octo-tree with the "octomap" C++ library.
		 *  This class represents a 3D map where each voxel only contains an "occupancy" property.
		 *
		 * As with any other mrpt::slam::CMetricMap, you can obtain a 3D representation of the map calling getAs3DObject() or getAsOctoMapVoxels()
		 *
		 * To use octomap's iterators to go through the voxels, use COctoMap::getOctomap()
		 *
		 * The octomap library was presented in:
		 *  - K. M. Wurm, A. Hornung, M. Bennewitz, C. Stachniss, and W. Burgard,
		 *     <i>"OctoMap: A Probabilistic, Flexible, and Compact 3D Map Representation for Robotic Systems"</i>
		 *     in Proc. of the ICRA 2010 Workshop on Best Practice in 3D Perception and Modeling for Mobile Manipulation, 2010. Software available at http://octomap.sf.net/.
		 *
		 * \sa CMetricMap, the example in "MRPT/samples/octomap_simple"
	  	 * \ingroup mrpt_maps_grp
		 */
		class MAPS_IMPEXP COctoMap : public CMetricMap
		{
			// This must be added to any CSerializable derived class:
			DEFINE_SERIALIZABLE( COctoMap )

		 public:
			 COctoMap(const double resolution=0.10);          //!< Default constructor
			 virtual ~COctoMap(); //!< Destructor

			 /** Get a reference to the internal octomap object. Example:
			   * \code
			   *  mrpt::maps::COctoMap  map;
			   *  ...
			   *  octomap::OcTree &om = map.getOctomap();
			   *
			   *
			   * \endcode
			   */
			 octomap::OcTree & getOctomap() { return *m_octomap; }

			/** With this struct options are provided to the observation insertion process.
			* \sa CObservation::insertObservationInto()
			*/
			struct MAPS_IMPEXP TInsertionOptions : public utils::CLoadableOptions
			{
				/** Initilization of default parameters */
				TInsertionOptions( COctoMap &parent );

				TInsertionOptions(); //!< Especial constructor, not attached to a real COctoMap object: used only in limited situations, since get*() methods don't work, etc.
				TInsertionOptions & operator = (const TInsertionOptions &other);

				/** See utils::CLoadableOptions */
				void  loadFromConfigFile(const mrpt::utils::CConfigFileBase  &source,const std::string &section);
				/** See utils::CLoadableOptions */
				void  dumpToTextStream(CStream	&out) const;

				double maxrange;  //!< maximum range for how long individual beams are inserted (default -1: complete beam)
				bool pruning;     //!< whether the tree is (losslessly) pruned after insertion (default: true)

				/// (key name in .ini files: "occupancyThres") sets the threshold for occupancy (sensor model) (Default=0.5)
				void setOccupancyThres(double prob);
				/// (key name in .ini files: "probHit")sets the probablility for a "hit" (will be converted to logodds) - sensor model (Default=0.7)
				void setProbHit(double prob);
				/// (key name in .ini files: "probMiss")sets the probablility for a "miss" (will be converted to logodds) - sensor model (Default=0.4)
				void setProbMiss(double prob);
				/// (key name in .ini files: "clampingThresMin")sets the minimum threshold for occupancy clamping (sensor model) (Default=0.1192, -2 in log odds)
				void setClampingThresMin(double thresProb);
				/// (key name in .ini files: "clampingThresMax")sets the maximum threshold for occupancy clamping (sensor model) (Default=0.971, 3.5 in log odds)
				void setClampingThresMax(double thresProb);

				/// @return threshold (probability) for occupancy - sensor model
				double getOccupancyThres() const;
				/// @return threshold (logodds) for occupancy - sensor model
				float getOccupancyThresLog() const;

				/// @return probablility for a "hit" in the sensor model (probability)
				double getProbHit() const;
				/// @return probablility for a "hit" in the sensor model (logodds)
				float getProbHitLog() const;
				/// @return probablility for a "miss"  in the sensor model (probability)
				double getProbMiss() const;
				/// @return probablility for a "miss"  in the sensor model (logodds)
				float getProbMissLog() const;

				/// @return minimum threshold for occupancy clamping in the sensor model (probability)
				double getClampingThresMin() const;
				/// @return minimum threshold for occupancy clamping in the sensor model (logodds)
				float getClampingThresMinLog() const;
				/// @return maximum threshold for occupancy clamping in the sensor model (probability)
				double getClampingThresMax() const;
				/// @return maximum threshold for occupancy clamping in the sensor model (logodds)
				float getClampingThresMaxLog() const;

			private:
				mrpt::utils::ignored_copy_ptr<COctoMap> m_parent;

				double occupancyThres; // sets the threshold for occupancy (sensor model) (Default=0.5)
				double probHit; // sets the probablility for a "hit" (will be converted to logodds) - sensor model (Default=0.7)
				double probMiss; // sets the probablility for a "miss" (will be converted to logodds) - sensor model (Default=0.4)
				double clampingThresMin; // sets the minimum threshold for occupancy clamping (sensor model) (Default=0.1192, -2 in log odds)
				double clampingThresMax; // sets the maximum threshold for occupancy clamping (sensor model) (Default=0.971, 3.5 in log odds)
			};

			TInsertionOptions insertionOptions; //!< The options used when inserting observations in the map

			friend struct mrpt::slam::COctoMap::TInsertionOptions;

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

				uint32_t	decimation; //!< Speed up the likelihood computation by considering only one out of N rays (default=1)
			};

			TLikelihoodOptions  likelihoodOptions;

			/** Returns true if the map is empty/no observation has been inserted.
				*/
			virtual bool  isEmpty() const;


			/** Computes the log-likelihood of a given observation given an arbitrary robot 3D pose.
				* In this particular class, the log-likelihood is computed by adding (product of likelihoods)
				* the logarithm of the occupancy probability of all cells in which an endpoint from the sensor rays.
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
			virtual void  saveMetricMapRepresentationToFile(const std::string	&filNamePrefix) const;

			/** Options for the conversion of a mrpt::slam::COctoMap into a mrpt::opengl::COctoMapVoxels */
			struct MAPS_IMPEXP TRenderingOptions
			{
				bool  generateGridLines;       //!< Generate grid lines for all octree nodes, useful to draw the "structure" of the octree, but computationally costly (Default: false)

				bool  generateOccupiedVoxels;  //!< Generate voxels for the occupied volumes  (Default=true)
				bool  visibleOccupiedVoxels;   //!< Set occupied voxels visible (requires generateOccupiedVoxels=true) (Default=true)

				bool  generateFreeVoxels;      //!< Generate voxels for the freespace (Default=true)
				bool  visibleFreeVoxels;       //!< Set free voxels visible (requires generateFreeVoxels=true) (Default=true)

				TRenderingOptions();

				void writeToStream(CStream &out) const;		//!< Binary dump to stream
				void readFromStream(CStream &in);			//!< Binary dump to stream
			};

			TRenderingOptions renderingOptions;

			/** Returns a 3D object representing the map.
				* \sa renderingOptions
				*/
			virtual void  getAs3DObject( mrpt::opengl::CSetOfObjectsPtr	&outObj ) const;

			/** Builds a renderizable representation of the octomap as a mrpt::opengl::COctoMapVoxels object.
				* \sa renderingOptions
				*/
			void getAsOctoMapVoxels(mrpt::opengl::COctoMapVoxels &gl_obj) const;

			/** Check whether the given point lies within the volume covered by the octomap (that is, whether it is "mapped") */
			bool isPointWithinOctoMap(const float x,const float y,const float z) const;

			/** Get the occupancy probability [0,1] of a point
				* \return false if the point is not mapped, in which case the returned "prob" is undefined. */
			bool getPointOccupancy(const float x,const float y,const float z, double &prob_occupancy) const;

			/** Manually updates the occupancy of the voxel at (x,y,z) as being occupied (true) or free (false), using the log-odds parameters in \a insertionOptions */
			void updateVoxel(const double x, const double y, const double z, bool occupied);

			/** Update the octomap with a 2D or 3D scan, given directly as a point cloud and the 3D location of the sensor (the origin of the rays) in this map's frame of reference.
			  * Insertion parameters can be found in \a insertionOptions.
			  * \sa The generic observation insertion method CMetricMap::insertObservation()
			  */
			void insertPointCloud(const CPointsMap &ptMap, const float sensor_x,const float sensor_y,const float sensor_z);

			/** Just like insertPointCloud but with a single ray. */
			void insertRay(const float end_x,const float end_y,const float end_z,const float sensor_x,const float sensor_y,const float sensor_z);

			/** Performs raycasting in 3d, similar to computeRay().
			 *
			 * A ray is cast from origin with a given direction, the first occupied
			 * cell is returned (as center coordinate). If the starting coordinate is already
			 * occupied in the tree, this coordinate will be returned as a hit.
			 *
			 * @param origin starting coordinate of ray
			 * @param direction A vector pointing in the direction of the raycast. Does not need to be normalized.
			 * @param end returns the center of the cell that was hit by the ray, if successful
			 * @param ignoreUnknownCells whether unknown cells are ignored. If false (default), the raycast aborts when an unkown cell is hit.
			 * @param maxRange Maximum range after which the raycast is aborted (<= 0: no limit, default)
			 * @return whether or not an occupied cell was hit
			 */
			bool castRay(
				const mrpt::math::TPoint3D & origin,
				const mrpt::math::TPoint3D & direction,
				mrpt::math::TPoint3D & end,
				bool ignoreUnknownCells=false,
				double maxRange=-1.0) const;

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

			/**  Builds the list of 3D points in global coordinates for a generic observation. Used for both, insertObservation() and computeLikelihood().
			  * \param[out] point3d_sensorPt Is a pointer to a "point3D".
			  * \param[out] ptr_scan Is in fact a pointer to "octomap::Pointcloud". Not declared as such to avoid headers dependencies in user code.
			  * \return false if the observation kind is not applicable.
			  */
			bool internal_build_PointCloud_for_observation(const CObservation *obs,const CPose3D *robotPose, void *point3d_sensorPt, void *ptr_scan) const;

			void freeOctomap();  //!< Internal use only
			void allocOctomap(double resolution); //!< Internal use only

			octomap::OcTree *m_octomap; //!< The pointer to the actual octo-map object.

		}; // End of class def.
	} // End of namespace

} // End of namespace

#endif
