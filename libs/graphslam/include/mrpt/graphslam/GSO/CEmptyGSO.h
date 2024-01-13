/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/graphslam/interfaces/CGraphSlamOptimizer.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CSensoryFrame.h>

namespace mrpt::graphslam::optimizers
{
/**\brief Empty Edge Registration Decider
 *
 * Handy when you are testing other parts of the application but not the
 * specific registration procedure
 */
template <class GRAPH_T = typename mrpt::graphs::CNetworkOfPoses2DInf>
class CEmptyGSO
	: public mrpt::graphslam::optimizers::CGraphSlamOptimizer<GRAPH_T>
{
   public:
	using constraint_t = typename GRAPH_T::constraint_t;

	CEmptyGSO() = default;
	~CEmptyGSO() = default;

	bool updateState(
		mrpt::obs::CActionCollection::Ptr action,
		mrpt::obs::CSensoryFrame::Ptr observations,
		mrpt::obs::CObservation::Ptr observation)
	{
		return true;
	}

   private:
	void registerNewEdge(
		const mrpt::graphs::TNodeID& from, const mrpt::graphs::TNodeID& to,
		const constraint_t& rel_edge)
	{
	}
};
}  // namespace mrpt::graphslam::optimizers
