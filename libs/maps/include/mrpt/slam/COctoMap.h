/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef MRPT_COctoMap_H
#define MRPT_COctoMap_H

#include <mrpt/slam/COctoMapBase.h>

#include <mrpt/slam/CMetricMap.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils/safe_pointers.h>
#include <mrpt/otherlibs/octomap/octomap.h>

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
		 *  This version only stores occupancy information at each octree node. See the base class mrpt::slam::COctoMapBase.
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

			virtual void getAsOctoMapVoxels(mrpt::opengl::COctoMapVoxels &gl_obj) const;

		protected:
			bool internal_insertObservation(const CObservation *obs,const CPose3D *robotPose);

		}; // End of class def.
		DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( COctoMap , CMetricMap, MAPS_IMPEXP )
	} // End of namespace

} // End of namespace

#endif
