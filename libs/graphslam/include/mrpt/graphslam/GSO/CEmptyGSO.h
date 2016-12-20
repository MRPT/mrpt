	/* +---------------------------------------------------------------------------+
	 |                     Mobile Robot Programming Toolkit (MRPT)               |
	 |                          http://www.mrpt.org/                             |
	 |                                                                           |
	 | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
	 | See: http://www.mrpt.org/Authors - All rights reserved.                   |
	 | Released under BSD License. See details in http://www.mrpt.org/License    |
	 +---------------------------------------------------------------------------+ */

#ifndef CEmptyGSO_H
#define CEmptyGSO_H

#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/CRawlog.h>

#include <mrpt/graphslam/interfaces/CGraphSlamOptimizer.h>

namespace mrpt { namespace graphslam { namespace optimizers {

/**\brief Empty Edge Registration Decider
 *
 * Handy when you are testing other parts of the application but not the
 * specific registration procedure
 */
template<class GRAPH_t=typename mrpt::graphs::CNetworkOfPoses2DInf >
class CEmptyGSO:
	public mrpt::graphslam::optimizers::CGraphSlamOptimizer<GRAPH_t>
{
	public:
		typedef typename GRAPH_t::constraint_t constraint_t;

		CEmptyGSO();
		~CEmptyGSO();

		bool updateState(
				mrpt::obs::CActionCollectionPtr action,
				mrpt::obs::CSensoryFramePtr observations,
				mrpt::obs::CObservationPtr observation );

	private:
		void registerNewEdge(
				const mrpt::utils::TNodeID& from,
				const mrpt::utils::TNodeID& to,
				const constraint_t& rel_edge );
};

//////////////////////////////////////////////////////////////////////////////

template<class GRAPH_t>
CEmptyGSO<GRAPH_t>::CEmptyGSO() { }
template<class GRAPH_t>
CEmptyGSO<GRAPH_t>::~CEmptyGSO() { }

template<class GRAPH_t>
bool CEmptyGSO<GRAPH_t>::updateState(
		mrpt::obs::CActionCollectionPtr action,
		mrpt::obs::CSensoryFramePtr observations,
		mrpt::obs::CObservationPtr observation ) {return true;}

template<class GRAPH_t>
void CEmptyGSO<GRAPH_t>::registerNewEdge(
		const mrpt::utils::TNodeID& from,
		const mrpt::utils::TNodeID& to,
		const constraint_t& rel_edge ) { }

} } } // end of namespaces

#endif /* end of include guard: CEmptyGSO_H */
