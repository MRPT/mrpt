/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#ifndef CEmptyERD_H
#define CEmptyERD_H

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

	CEmptyERD();
	~CEmptyERD();

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
CEmptyERD<GRAPH_T>::CEmptyERD()
{
}
template <class GRAPH_T>
CEmptyERD<GRAPH_T>::~CEmptyERD()
{
}

template <class GRAPH_T>
bool CEmptyERD<GRAPH_T>::updateState(
	mrpt::obs::CActionCollection::Ptr action,
	mrpt::obs::CSensoryFrame::Ptr observations,
	mrpt::obs::CObservation::Ptr observation)
{
	return true;
}

template <class GRAPH_T>
void CEmptyERD<GRAPH_T>::registerNewEdge(
	const mrpt::graphs::TNodeID& from, const mrpt::graphs::TNodeID& to,
	const constraint_t& rel_edge)
{
}
}  // end of namespaces

#endif /* end of include guard: CEmptyERD_H */


