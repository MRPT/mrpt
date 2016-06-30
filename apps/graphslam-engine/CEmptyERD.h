	/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CEmptyERD_H
#define CEmptyERD_H

#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/CRawlog.h>


#include "CEdgeRegistrationDecider.h"

namespace mrpt { namespace graphslam { namespace deciders {

/**
 * Empty Registration Decider
 *
 * Handy when you have are testing other parts of the application but not the
 * specific registration procedure
 */
template< class GRAPH_t=typename mrpt::graphs::CNetworkOfPoses2DInf >
class CEmptyERD_t:
	public mrpt::graphslam::deciders::CEdgeRegistrationDecider_t<GRAPH_t> 
{
	public:
		typedef typename GRAPH_t::constraint_t constraint_t;
		typedef typename GRAPH_t::constraint_t::type_value pose_t; // type of underlying poses (2D/3D)

	  CEmptyERD_t();
	  ~CEmptyERD_t();

		void updateDeciderState(
				mrpt::obs::CActionCollectionPtr action,
				mrpt::obs::CSensoryFramePtr observations,
				mrpt::obs::CObservationPtr observation );

	private:
    void registerNewEdge(
    		const mrpt::utils::TNodeID& from, 
    		const mrpt::utils::TNodeID& to,
    		const constraint_t& rel_edge );
};

} } } // end of namespaces

using namespace mrpt::graphslam::deciders;

template<class GRAPH_t>
CEmptyERD_t<GRAPH_t>::CEmptyERD_t() { }
template<class GRAPH_t>
CEmptyERD_t<GRAPH_t>::~CEmptyERD_t() { }

template<class GRAPH_t> 
void CEmptyERD_t<GRAPH_t>::updateDeciderState(
		mrpt::obs::CActionCollectionPtr action,
		mrpt::obs::CSensoryFramePtr observations,
		mrpt::obs::CObservationPtr observation ) { }

template<class GRAPH_t>
void CEmptyERD_t<GRAPH_t>::registerNewEdge(
    const mrpt::utils::TNodeID& from, 
    const mrpt::utils::TNodeID& to,
    const constraint_t& rel_edge ) { }

#endif /* end of include guard: CEmptyERD_H */
