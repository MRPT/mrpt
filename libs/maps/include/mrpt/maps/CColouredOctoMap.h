/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef MRPT_CColouredOctoMap_H
#define MRPT_CColouredOctoMap_H

#include <mrpt/maps/COctoMapBase.h>
#include <mrpt/obs/obs_frwds.h>

#include <mrpt/maps/link_pragmas.h>

PIMPL_FORWARD_DECLARATION(namespace octomap { class ColorOcTree; })
PIMPL_FORWARD_DECLARATION(namespace octomap { class ColorOcTreeNode; })

namespace mrpt
{
	namespace maps
	{
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CColouredOctoMap , CMetricMap, MAPS_IMPEXP )

		/** A three-dimensional probabilistic occupancy grid, implemented as an octo-tree with the "octomap" C++ library.
		 *  This version stores both, occupancy information and RGB colour data at each octree node. See the base class mrpt::maps::COctoMapBase.
		 *
		 * \sa CMetricMap, the example in "MRPT/samples/octomap_simple"
	  	 * \ingroup mrpt_maps_grp
		 */
		class MAPS_IMPEXP CColouredOctoMap : public COctoMapBase<octomap::ColorOcTree,octomap::ColorOcTreeNode>
		{
			// This must be added to any CSerializable derived class:
			DEFINE_SERIALIZABLE( CColouredOctoMap )

		 public:
			 CColouredOctoMap(const double resolution=0.10);          //!< Default constructor
			 virtual ~CColouredOctoMap(); //!< Destructor

			/** This allows the user to select the desired method to update voxels colour.
				SET = Set the colour of the voxel at (x,y,z) directly
				AVERAGE = Set the colour of the voxel at (x,y,z) as the mean of its previous colour and the new observed one.
				INTEGRATE = Calculate the new colour of the voxel at (x,y,z) using this formula: prev_color*node_prob +  new_color*(0.99-node_prob)
				If there isn't any previous color, any method is equivalent to SET.
				INTEGRATE is the default option*/
			enum TColourUpdate
			{
				INTEGRATE = 0,
				SET,
				AVERAGE
			};

			/** Get the RGB colour of a point
				* \return false if the point is not mapped, in which case the returned colour is undefined. */
			bool getPointColour(const float x, const float y, const float z, uint8_t& r, uint8_t& g, uint8_t& b) const;

			/** Manually update the colour of the voxel at (x,y,z) */
			void updateVoxelColour(const double x, const double y, const double z, const uint8_t r, const uint8_t g, const uint8_t b);

			///Set the method used to update voxels colour
			void setVoxelColourMethod(TColourUpdate new_method) {m_colour_method = new_method;}

			///Get the method used to update voxels colour
			TColourUpdate getVoxelColourMethod() {return m_colour_method;}

			virtual void getAsOctoMapVoxels(mrpt::opengl::COctoMapVoxels &gl_obj) const MRPT_OVERRIDE;

			MAP_DEFINITION_START(CColouredOctoMap,MAPS_IMPEXP)
				double resolution;	//!< The finest resolution of the octomap (default: 0.10 meters)
				mrpt::maps::CColouredOctoMap::TInsertionOptions   insertionOpts;	//!< Observations insertion options
				mrpt::maps::CColouredOctoMap::TLikelihoodOptions  likelihoodOpts;	//!< Probabilistic observation likelihood options
				MAP_DEFINITION_END(CColouredOctoMap, MAPS_IMPEXP)

			/** Returns true if the map is empty/no observation has been inserted */
			virtual bool isEmpty() const MRPT_OVERRIDE { return size() == 1; }

			/** @name Direct access to octomap library methods
			@{ */

			/** Just like insertPointCloud but with a single ray. */
			void insertRay(const float end_x, const float end_y, const float end_z, const float sensor_x, const float sensor_y, const float sensor_z);
			/** Manually updates the occupancy of the voxel at (x,y,z) as being occupied (true) or free (false), using the log-odds parameters in \a insertionOptions */
			void updateVoxel(const double x, const double y, const double z, bool occupied);
			/** Check whether the given point lies within the volume covered by the octomap (that is, whether it is "mapped") */
			bool isPointWithinOctoMap(const float x, const float y, const float z) const;
			double getResolution() const;
			unsigned int getTreeDepth() const;
			/// \return The number of nodes in the tree
			size_t size() const;
			/// \return Memory usage of the complete octree in bytes (may vary between architectures)
			size_t memoryUsage() const;
			/// \return Memory usage of the a single octree node
			size_t memoryUsageNode() const;
			/// \return Memory usage of a full grid of the same size as the OcTree in bytes (for comparison)
			size_t memoryFullGrid() const;
			double volume();
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

			void setOccupancyThres(double prob) MRPT_OVERRIDE;
			void setProbHit(double prob) MRPT_OVERRIDE;
			void setProbMiss(double prob) MRPT_OVERRIDE;
			void setClampingThresMin(double thresProb) MRPT_OVERRIDE;
			void setClampingThresMax(double thresProb) MRPT_OVERRIDE;
			double getOccupancyThres() const MRPT_OVERRIDE;
			float getOccupancyThresLog() const MRPT_OVERRIDE;
			double getProbHit() const MRPT_OVERRIDE;
			float getProbHitLog() const MRPT_OVERRIDE;
			double getProbMiss() const MRPT_OVERRIDE;
			float getProbMissLog() const MRPT_OVERRIDE;
			double getClampingThresMin() const MRPT_OVERRIDE;
			float getClampingThresMinLog() const MRPT_OVERRIDE;
			double getClampingThresMax() const MRPT_OVERRIDE;
			float getClampingThresMaxLog() const MRPT_OVERRIDE;
			/** @} */

		protected:
			virtual void  internal_clear() MRPT_OVERRIDE;

			bool internal_insertObservation(const mrpt::obs::CObservation *obs,const mrpt::poses::CPose3D *robotPose) MRPT_OVERRIDE;

			TColourUpdate m_colour_method;		//!Method used to updated voxels colour.

		}; // End of class def.
		DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CColouredOctoMap , CMetricMap, MAPS_IMPEXP )

	} // End of namespace

} // End of namespace

#endif
