/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

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
/** A three-dimensional probabilistic occupancy grid, implemented as an
 * octo-tree with the "octomap" C++ library.
 *  This version only stores occupancy information at each octree node. See the
 * base class mrpt::maps::COctoMapBase.
 *
 * \sa CMetricMap, the example in "MRPT/samples/octomap_simple"
 * \ingroup mrpt_maps_grp
 */
class MAPS_IMPEXP COctoMap
	: public COctoMapBase<octomap::OcTree, octomap::OcTreeNode>
{
	DEFINE_SERIALIZABLE(COctoMap)

   public:
	/** Default constructor */
	COctoMap(const double resolution = 0.10);
	/** Destructor */
	virtual ~COctoMap();

	virtual void getAsOctoMapVoxels(
		mrpt::opengl::COctoMapVoxels& gl_obj) const override;

	MAP_DEFINITION_START(COctoMap, MAPS_IMPEXP)
	/** The finest resolution of the octomap (default: 0.10 meters) */
	double resolution;
	/** Observations insertion options */
	mrpt::maps::COctoMap::TInsertionOptions insertionOpts;
	/** Probabilistic observation likelihood options */
	mrpt::maps::COctoMap::TLikelihoodOptions likelihoodOpts;
	MAP_DEFINITION_END(COctoMap, MAPS_IMPEXP)

   protected:
	bool internal_insertObservation(
		const mrpt::obs::CObservation* obs,
		const mrpt::poses::CPose3D* robotPose) override;
};  // End of class def.
DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE(COctoMap, CMetricMap, MAPS_IMPEXP)
}  // End of namespace

}  // End of namespace

#endif
