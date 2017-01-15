	/* +---------------------------------------------------------------------------+
	 |                     Mobile Robot Programming Toolkit (MRPT)               |
	 |                          http://www.mrpt.org/                             |
	 |                                                                           |
	 | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
	 | See: http://www.mrpt.org/Authors - All rights reserved.                   |
	 | Released under BSD License. See details in http://www.mrpt.org/License    |
	 +---------------------------------------------------------------------------+ */

#ifndef CEmptyERD_H
#define CEmptyERD_H

#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/CRawlog.h>

#include <mrpt/graphslam/interfaces/CEdgeRegistrationDecider.h>

namespace mrpt { namespace graphslam { namespace deciders {

/**\brief Empty Edge Registration Decider
 *
 * Handy when you are testing other parts of the application but not the
 * specific registration procedure
 */
template<class GRAPH_t=typename mrpt::graphs::CNetworkOfPoses2DInf >
class CEmptyERD:
	public mrpt::graphslam::deciders::CEdgeRegistrationDecider<GRAPH_t>
{
	public:
		typedef typename GRAPH_t::constraint_t constraint_t;

		CEmptyERD();
		~CEmptyERD();

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
CEmptyERD<GRAPH_t>::CEmptyERD() { }
template<class GRAPH_t>
CEmptyERD<GRAPH_t>::~CEmptyERD() { }

template<class GRAPH_t>
bool CEmptyERD<GRAPH_t>::updateState(
		mrpt::obs::CActionCollectionPtr action,
		mrpt::obs::CSensoryFramePtr observations,
		mrpt::obs::CObservationPtr observation ) {return true;}

template<class GRAPH_t>
void CEmptyERD<GRAPH_t>::registerNewEdge(
		const mrpt::utils::TNodeID& from,
		const mrpt::utils::TNodeID& to,
		const constraint_t& rel_edge ) { }

} } } // end of namespaces

#endif /* end of include guard: CEmptyERD_H */
