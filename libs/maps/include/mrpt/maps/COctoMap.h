/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef MRPT_COctoMap_H
#define MRPT_COctoMap_H

#include <mrpt/maps/COctoMapBase.h>

#include <mrpt/maps/CMetricMap.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils/safe_pointers.h>
#include <mrpt/otherlibs/octomap/octomap.h>
#include <mrpt/obs/obs_frwds.h>

#include <mrpt/maps/link_pragmas.h>

namespace mrpt
{
	namespace maps
	{
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( COctoMap , CMetricMap, MAPS_IMPEXP )

		/** A three-dimensional probabilistic occupancy grid, implemented as an octo-tree with the "octomap" C++ library.
		 *  This version only stores occupancy information at each octree node. See the base class mrpt::maps::COctoMapBase.
		 *
		 * \sa CMetricMap, the example in "MRPT/samples/octomap_simple"
	  	 * \ingroup mrpt_maps_grp
		 */
		class MAPS_IMPEXP COctoMap : public COctoMapBase<octomap::OcTree,octomap::OcTreeNode>
		{
			// This must be added to any CSerializable derived class:
			DEFINE_SERIALIZABLE( COctoMap )

		 public:
			 COctoMap(const double resolution=0.10);          //!< Default constructor
			 virtual ~COctoMap(); //!< Destructor

			virtual void getAsOctoMapVoxels(mrpt::opengl::COctoMapVoxels &gl_obj) const MRPT_OVERRIDE;

			MAP_DEFINITION_START(COctoMap,MAPS_IMPEXP)
				double resolution;	//!< The finest resolution of the octomap (default: 0.10 meters)
				mrpt::maps::COctoMap::TInsertionOptions   insertionOpts;	//!< Observations insertion options
				mrpt::maps::COctoMap::TLikelihoodOptions  likelihoodOpts;	//!< Probabilistic observation likelihood options
			MAP_DEFINITION_END(COctoMap,MAPS_IMPEXP)

		protected:
			bool internal_insertObservation(const mrpt::obs::CObservation *obs,const mrpt::poses::CPose3D *robotPose) MRPT_OVERRIDE;
		}; // End of class def.
		DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( COctoMap , CMetricMap, MAPS_IMPEXP )
	} // End of namespace

} // End of namespace

#endif
