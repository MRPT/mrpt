/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#ifndef CEmptyGSO_H
#define CEmptyGSO_H

#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CSensoryFrame.h>

#include <mrpt/graphslam/interfaces/CGraphSlamOptimizer.h>

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

	CEmptyGSO();
	~CEmptyGSO();

	bool updateState(
		mrpt::obs::CActionCollection::Ptr action,
		mrpt::obs::CSensoryFrame::Ptr observations,
		mrpt::obs::CObservation::Ptr observation);

   private:
	void registerNewEdge(
		const mrpt::graphs::TNodeID& from, const mrpt::graphs::TNodeID& to,
		const constraint_t& rel_edge);
};

//////////////////////////////////////////////////////////////////////////////

template <class GRAPH_T>
CEmptyGSO<GRAPH_T>::CEmptyGSO()
{
}
template <class GRAPH_T>
CEmptyGSO<GRAPH_T>::~CEmptyGSO()
{
}

template <class GRAPH_T>
bool CEmptyGSO<GRAPH_T>::updateState(
	mrpt::obs::CActionCollection::Ptr action,
	mrpt::obs::CSensoryFrame::Ptr observations,
	mrpt::obs::CObservation::Ptr observation)
{
	return true;
}

template <class GRAPH_T>
void CEmptyGSO<GRAPH_T>::registerNewEdge(
	const mrpt::graphs::TNodeID& from, const mrpt::graphs::TNodeID& to,
	const constraint_t& rel_edge)
{
}
}  // end of namespaces

#endif /* end of include guard: CEmptyGSO_H */


