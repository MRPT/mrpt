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
#include <mrpt/otherlibs/octomap/octomap.h>
#include <mrpt/otherlibs/octomap/ColorOcTree.h>
#include <mrpt/obs/obs_frwds.h>

#include <mrpt/maps/link_pragmas.h>

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
			MAP_DEFINITION_END(CColouredOctoMap,MAPS_IMPEXP)

		protected:
			bool internal_insertObservation(const mrpt::obs::CObservation *obs,const mrpt::poses::CPose3D *robotPose) MRPT_OVERRIDE;

			TColourUpdate m_colour_method;		//!Method used to updated voxels colour.

		}; // End of class def.
		DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CColouredOctoMap , CMetricMap, MAPS_IMPEXP )

	} // End of namespace

} // End of namespace

#endif
