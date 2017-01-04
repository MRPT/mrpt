/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef MRPT_COctoMapBase_H
#define MRPT_COctoMapBase_H

#include <mrpt/maps/CMetricMap.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils/safe_pointers.h>
#include <mrpt/otherlibs/octomap/octomap.h>
#include <mrpt/opengl/COctoMapVoxels.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/obs/obs_frwds.h>

#include <mrpt/maps/link_pragmas.h>

namespace mrpt
{
	namespace maps
	{
		/** A three-dimensional probabilistic occupancy grid, implemented as an octo-tree with the "octomap" C++ library.
		 *  This base class represents a 3D map where each voxel may contain an "occupancy" property, RGBD data, etc. depending on the derived class.
		 *
		 * As with any other mrpt::maps::CMetricMap, you can obtain a 3D representation of the map calling getAs3DObject() or getAsOctoMapVoxels()
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
		template <class OCTREE,class OCTREE_NODE>
		class COctoMapBase : public mrpt::maps::CMetricMap
		{
		public:
			typedef COctoMapBase<OCTREE,OCTREE_NODE> myself_t;       //!< The type of this MRPT class
			typedef OCTREE                           octree_t;       //!< The type of the octree class in the "octomap" library.
			typedef OCTREE_NODE                      octree_node_t;  //!< The type of nodes in the octree, in the "octomap" library.


			/** Constructor, defines the resolution of the octomap (length of each voxel side) */
			COctoMapBase(const double resolution=0.10) : insertionOptions(*this), m_octomap(resolution) { }
			virtual ~COctoMapBase() { }

			/** Get a reference to the internal octomap object. Example:
			   * \code
			   *  mrpt::maps::COctoMap  map;
			   *  ...
			   *  octomap::OcTree &om = map.getOctomap();
			   *
			   *
			   * \endcode
			   */
			inline OCTREE & getOctomap() { return m_octomap; }

			/** With this struct options are provided to the observation insertion process.
			* \sa CObservation::insertObservationInto()
			*/
			struct TInsertionOptions : public utils::CLoadableOptions
			{
				/** Initilization of default parameters */
				TInsertionOptions( myself_t &parent );

				TInsertionOptions(); //!< Especial constructor, not attached to a real COctoMap object: used only in limited situations, since get*() methods don't work, etc.
				TInsertionOptions & operator = (const TInsertionOptions &o)
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

				void loadFromConfigFile(const mrpt::utils::CConfigFileBase &source,const std::string &section) MRPT_OVERRIDE; // See base docs
				void dumpToTextStream(mrpt::utils::CStream &out) const MRPT_OVERRIDE; // See base docs

				double maxrange;  //!< maximum range for how long individual beams are inserted (default -1: complete beam)
				bool pruning;     //!< whether the tree is (losslessly) pruned after insertion (default: true)

				/// (key name in .ini files: "occupancyThres") sets the threshold for occupancy (sensor model) (Default=0.5)
				void setOccupancyThres(double prob) { if(m_parent.get()) m_parent->m_octomap.setOccupancyThres(prob); }
				/// (key name in .ini files: "probHit")sets the probablility for a "hit" (will be converted to logodds) - sensor model (Default=0.7)
				void setProbHit(double prob) { if(m_parent.get()) m_parent->m_octomap.setProbHit(prob); }
				/// (key name in .ini files: "probMiss")sets the probablility for a "miss" (will be converted to logodds) - sensor model (Default=0.4)
				void setProbMiss(double prob) { if(m_parent.get()) m_parent->m_octomap.setProbMiss(prob); }
				/// (key name in .ini files: "clampingThresMin")sets the minimum threshold for occupancy clamping (sensor model) (Default=0.1192, -2 in log odds)
				void setClampingThresMin(double thresProb) { if(m_parent.get()) m_parent->m_octomap.setClampingThresMin(thresProb); }
				/// (key name in .ini files: "clampingThresMax")sets the maximum threshold for occupancy clamping (sensor model) (Default=0.971, 3.5 in log odds)
				void setClampingThresMax(double thresProb) { if(m_parent.get()) m_parent->m_octomap.setClampingThresMax(thresProb); }

				/// @return threshold (probability) for occupancy - sensor model
				double getOccupancyThres() const { if(m_parent.get()) return m_parent->m_octomap.getOccupancyThres(); else return this->occupancyThres; }
				/// @return threshold (logodds) for occupancy - sensor model
				float getOccupancyThresLog() const { return m_parent->m_octomap.getOccupancyThresLog() ; }

				/// @return probablility for a "hit" in the sensor model (probability)
				double getProbHit() const { if(m_parent.get()) return m_parent->m_octomap.getProbHit(); else return this->probHit; }
				/// @return probablility for a "hit" in the sensor model (logodds)
				float getProbHitLog() const { return m_parent->m_octomap.getProbHitLog(); }
				/// @return probablility for a "miss"  in the sensor model (probability)
				double getProbMiss() const { if(m_parent.get()) return m_parent->m_octomap.getProbMiss(); else return this->probMiss; }
				/// @return probablility for a "miss"  in the sensor model (logodds)
				float getProbMissLog() const { return m_parent->m_octomap.getProbMissLog(); }

				/// @return minimum threshold for occupancy clamping in the sensor model (probability)
				double getClampingThresMin() const { if(m_parent.get()) return m_parent->m_octomap.getClampingThresMin(); else return this->clampingThresMin; }
				/// @return minimum threshold for occupancy clamping in the sensor model (logodds)
				float getClampingThresMinLog() const { return m_parent->m_octomap.getClampingThresMinLog(); }
				/// @return maximum threshold for occupancy clamping in the sensor model (probability)
				double getClampingThresMax() const { if(m_parent.get()) return m_parent->m_octomap.getClampingThresMax(); else return this->clampingThresMax; }
				/// @return maximum threshold for occupancy clamping in the sensor model (logodds)
				float getClampingThresMaxLog() const { return m_parent->m_octomap.getClampingThresMaxLog(); }

			private:
				mrpt::utils::ignored_copy_ptr<myself_t> m_parent;

				double occupancyThres; // sets the threshold for occupancy (sensor model) (Default=0.5)
				double probHit; // sets the probablility for a "hit" (will be converted to logodds) - sensor model (Default=0.7)
				double probMiss; // sets the probablility for a "miss" (will be converted to logodds) - sensor model (Default=0.4)
				double clampingThresMin; // sets the minimum threshold for occupancy clamping (sensor model) (Default=0.1192, -2 in log odds)
				double clampingThresMax; // sets the maximum threshold for occupancy clamping (sensor model) (Default=0.971, 3.5 in log odds)
			};

			TInsertionOptions insertionOptions; //!< The options used when inserting observations in the map

			/** Options used when evaluating "computeObservationLikelihood"
			* \sa CObservation::computeObservationLikelihood
			*/
			struct TLikelihoodOptions: public utils::CLoadableOptions
			{
				/** Initilization of default parameters
					*/
				TLikelihoodOptions( );
				virtual ~TLikelihoodOptions() {}

				void loadFromConfigFile(const mrpt::utils::CConfigFileBase &source,const std::string &section) MRPT_OVERRIDE; // See base docs
				void dumpToTextStream(mrpt::utils::CStream &out) const MRPT_OVERRIDE; // See base docs

				void writeToStream(mrpt::utils::CStream &out) const;		//!< Binary dump to stream
				void readFromStream(mrpt::utils::CStream &in);			//!< Binary dump to stream

				uint32_t	decimation; //!< Speed up the likelihood computation by considering only one out of N rays (default=1)
			};

			TLikelihoodOptions  likelihoodOptions;

			/** Returns true if the map is empty/no observation has been inserted */
			virtual bool isEmpty() const MRPT_OVERRIDE;

			virtual void  saveMetricMapRepresentationToFile(const std::string	&filNamePrefix) const MRPT_OVERRIDE;

			/** Options for the conversion of a mrpt::maps::COctoMap into a mrpt::opengl::COctoMapVoxels */
			struct TRenderingOptions
			{
				bool  generateGridLines;       //!< Generate grid lines for all octree nodes, useful to draw the "structure" of the octree, but computationally costly (Default: false)

				bool  generateOccupiedVoxels;  //!< Generate voxels for the occupied volumes  (Default=true)
				bool  visibleOccupiedVoxels;   //!< Set occupied voxels visible (requires generateOccupiedVoxels=true) (Default=true)

				bool  generateFreeVoxels;      //!< Generate voxels for the freespace (Default=true)
				bool  visibleFreeVoxels;       //!< Set free voxels visible (requires generateFreeVoxels=true) (Default=true)

				TRenderingOptions() :
					generateGridLines      (false),
					generateOccupiedVoxels (true),
					visibleOccupiedVoxels  (true),
					generateFreeVoxels     (true),
					visibleFreeVoxels      (true)
				{ }

				void writeToStream(mrpt::utils::CStream &out) const;		//!< Binary dump to stream
				void readFromStream(mrpt::utils::CStream &in);			//!< Binary dump to stream
			};

			TRenderingOptions renderingOptions;

			/** Returns a 3D object representing the map.
				* \sa renderingOptions
				*/
			virtual void  getAs3DObject( mrpt::opengl::CSetOfObjectsPtr	&outObj ) const MRPT_OVERRIDE
			{
				mrpt::opengl::COctoMapVoxelsPtr gl_obj = mrpt::opengl::COctoMapVoxels::Create();
				this->getAsOctoMapVoxels(*gl_obj);
				outObj->insert(gl_obj);
			}

			/** Builds a renderizable representation of the octomap as a mrpt::opengl::COctoMapVoxels object.
				* \sa renderingOptions
				*/
			virtual void getAsOctoMapVoxels(mrpt::opengl::COctoMapVoxels &gl_obj) const = 0;

			/** Check whether the given point lies within the volume covered by the octomap (that is, whether it is "mapped") */
			bool isPointWithinOctoMap(const float x,const float y,const float z) const
			{
				octomap::OcTreeKey key;
				return m_octomap.coordToKeyChecked(octomap::point3d(x,y,z), key);
			}

			/** Get the occupancy probability [0,1] of a point
				* \return false if the point is not mapped, in which case the returned "prob" is undefined. */
			bool getPointOccupancy(const float x,const float y,const float z, double &prob_occupancy) const;

			/** Manually updates the occupancy of the voxel at (x,y,z) as being occupied (true) or free (false), using the log-odds parameters in \a insertionOptions */
			void updateVoxel(const double x, const double y, const double z, bool occupied)
			{
				m_octomap.updateNode(x,y,z, occupied);
			}

			/** Update the octomap with a 2D or 3D scan, given directly as a point cloud and the 3D location of the sensor (the origin of the rays) in this map's frame of reference.
			  * Insertion parameters can be found in \a insertionOptions.
			  * \sa The generic observation insertion method CMetricMap::insertObservation()
			  */
			void insertPointCloud(const CPointsMap &ptMap, const float sensor_x,const float sensor_y,const float sensor_z);

			/** Just like insertPointCloud but with a single ray. */
			void insertRay(const float end_x,const float end_y,const float end_z,const float sensor_x,const float sensor_y,const float sensor_z)
			{
				m_octomap.insertRay( octomap::point3d(sensor_x,sensor_y,sensor_z), octomap::point3d(end_x,end_y,end_z), insertionOptions.maxrange,insertionOptions.pruning);
			}

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

			double getResolution() const { return m_octomap.getResolution(); }
			unsigned int getTreeDepth () const { return m_octomap.getTreeDepth(); }
			/// \return The number of nodes in the tree
			size_t size() const { return  m_octomap.size(); }
			/// \return Memory usage of the complete octree in bytes (may vary between architectures)
			size_t memoryUsage() const { return  m_octomap.memoryUsage(); }
			/// \return Memory usage of the a single octree node
			size_t memoryUsageNode() const { return  m_octomap.memoryUsageNode(); }
			/// \return Memory usage of a full grid of the same size as the OcTree in bytes (for comparison)
			size_t memoryFullGrid() const { return  m_octomap.memoryFullGrid(); }
			double volume() const { return m_octomap.volume(); }
			/// Size of OcTree (all known space) in meters for x, y and z dimension
			void getMetricSize(double& x, double& y, double& z) { return  m_octomap.getMetricSize(x,y,z); }
			/// Size of OcTree (all known space) in meters for x, y and z dimension
			void getMetricSize(double& x, double& y, double& z) const { return  m_octomap.getMetricSize(x,y,z); }
			/// minimum value of the bounding box of all known space in x, y, z
			void getMetricMin(double& x, double& y, double& z) { return  m_octomap.getMetricMin(x,y,z); }
			/// minimum value of the bounding box of all known space in x, y, z
			void getMetricMin(double& x, double& y, double& z) const { return  m_octomap.getMetricMin(x,y,z); }
			/// maximum value of the bounding box of all known space in x, y, z
			void getMetricMax(double& x, double& y, double& z) { return  m_octomap.getMetricMax(x,y,z); }
			/// maximum value of the bounding box of all known space in x, y, z
			void getMetricMax(double& x, double& y, double& z) const { return  m_octomap.getMetricMax(x,y,z); }

			/// Traverses the tree to calculate the total number of nodes
			size_t calcNumNodes() const { return  m_octomap.calcNumNodes(); }

			/// Traverses the tree to calculate the total number of leaf nodes
			size_t getNumLeafNodes() const { return  m_octomap.getNumLeafNodes(); }

			/** @} */


		protected:
			virtual void  internal_clear() MRPT_OVERRIDE {  m_octomap.clear(); }

			/**  Builds the list of 3D points in global coordinates for a generic observation. Used for both, insertObservation() and computeLikelihood().
			  * \param[out] point3d_sensorPt Is a pointer to a "point3D".
			  * \param[out] ptr_scan Is in fact a pointer to "octomap::Pointcloud". Not declared as such to avoid headers dependencies in user code.
			  * \return false if the observation kind is not applicable.
			  */
			bool internal_build_PointCloud_for_observation(const mrpt::obs::CObservation *obs,const mrpt::poses::CPose3D *robotPose, octomap::point3d &point3d_sensorPt, octomap::Pointcloud &ptr_scan) const;

			OCTREE m_octomap; //!< The actual octo-map object.

		private:
			// See docs in base class
			virtual double	 internal_computeObservationLikelihood( const mrpt::obs::CObservation *obs, const mrpt::poses::CPose3D &takenFrom ) MRPT_OVERRIDE;

		}; // End of class def.
	} // End of namespace
} // End of namespace

#include "COctoMapBase_impl.h"

#endif
