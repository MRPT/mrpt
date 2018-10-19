/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/obs/CSensoryFrame.h>

#include <mrpt/graphslam/interfaces/CEdgeRegistrationDecider.h>

namespace mrpt::graphslam::deciders
{
/**\brief Empty Edge Registration Decider
 *
 * Handy when you are testing other parts of the application but not the
 * specific registration procedure
 */
template <class GRAPH_T = typename mrpt::graphs::CNetworkOfPoses2DInf>
class CEmptyERD
	: public mrpt::graphslam::deciders::CEdgeRegistrationDecider<GRAPH_T>
{
   public:
	using constraint_t = typename GRAPH_T::constraint_t;

	CEmptyERD() = default;
	~CEmptyERD() override = default;

	bool updateState(
		mrpt::obs::CActionCollection::Ptr action,
		mrpt::obs::CSensoryFrame::Ptr observations,
		mrpt::obs::CObservation::Ptr observation) override
	{
		return true;
	}

   private:
	void registerNewEdge(
		const mrpt::graphs::TNodeID& from, const mrpt::graphs::TNodeID& to,
		const constraint_t& rel_edge) override
	{
	}
};
}  // namespace mrpt::graphslam::deciders
