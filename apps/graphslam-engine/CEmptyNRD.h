/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CEMPTYNRD_H
#define CEMPTYNRD_H

#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/CRawlog.h>

#include "CNodeRegistrationDecider.h"

namespace mrpt { namespace graphslam { namespace deciders {

/**
 * Empty Registration Decider
 *
 * Handy when you have are testing other parts of the application but not the
 * specific registration procedure
 */
template<class GRAPH_t=typename mrpt::graphs::CNetworkOfPoses2DInf>
class CEmptyNRD_t:
	public mrpt::graphslam::deciders::CNodeRegistrationDecider_t<GRAPH_t>
{
	public:
		CEmptyNRD_t();
		~CEmptyNRD_t();

		bool updateDeciderState( mrpt::obs::CActionCollectionPtr action,
				mrpt::obs::CSensoryFramePtr observations,
				mrpt::obs::CObservationPtr observation );

	private:
		void registerNewNode();

};

} } } // end of namespaces

using namespace mrpt::graphslam::deciders;

template<class GRAPH_t>
CEmptyNRD_t<GRAPH_t>::CEmptyNRD_t() { }
template<class GRAPH_t>
CEmptyNRD_t<GRAPH_t>::~CEmptyNRD_t() { }

template<class GRAPH_t>
bool CEmptyNRD_t<GRAPH_t>::updateDeciderState(
		mrpt::obs::CActionCollectionPtr action,
		mrpt::obs::CSensoryFramePtr observations,
		mrpt::obs::CObservationPtr observation )  {return false;}

template<class GRAPH_t>
void CEmptyNRD_t<GRAPH_t>::registerNewNode() { }

#endif /* end of include guard: CEMPTYNRD_H */
