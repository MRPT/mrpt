/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CSensoryFrame.h>

#include <mrpt/graphslam/interfaces/CNodeRegistrationDecider.h>

namespace mrpt::graphslam::deciders
{
/**\brief Empty Node Registration Decider
 *
 * Handy when you are testing other parts of the application but not the
 * specific registration procedure
 *
 * \ingroup mrpt_graphslam_grp
 */
template <class GRAPH_T = typename mrpt::graphs::CNetworkOfPoses2DInf>
class CEmptyNRD
	: public mrpt::graphslam::deciders::CNodeRegistrationDecider<GRAPH_T>
{
	/**\brief Handy typedefs */
	/**\{*/
	using pose_t = typename GRAPH_T::constraint_t::type_value;
	using global_pose_t = typename GRAPH_T::global_pose_t;
	/**\}*/
   public:
	CEmptyNRD() = default;
	~CEmptyNRD() override = default;

	bool updateState(
		mrpt::obs::CActionCollection::Ptr action,
		mrpt::obs::CSensoryFrame::Ptr observations,
		mrpt::obs::CObservation::Ptr observation) override
	{
		return false;
	}
	global_pose_t getCurrentRobotPosEstimation() const override
	{
		return typename GRAPH_T::global_pose_t();
	};

   private:
	void registerNewNode(){};
};
}  // namespace mrpt::graphslam::deciders
