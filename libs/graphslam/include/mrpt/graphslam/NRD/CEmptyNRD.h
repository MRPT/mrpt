/* +---------------------------------------------------------------------------+
	 |                     Mobile Robot Programming Toolkit (MRPT)               |
	 |                          http://www.mrpt.org/                             |
	 |                                                                           |
	 | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
	 | See: http://www.mrpt.org/Authors - All rights reserved.                   |
	 | Released under BSD License. See details in http://www.mrpt.org/License    |
	 +---------------------------------------------------------------------------+ */

#ifndef CEMPTYNRD_H
#define CEMPTYNRD_H

#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/CRawlog.h>

#include <mrpt/graphslam/interfaces/CNodeRegistrationDecider.h>

namespace mrpt { namespace graphslam { namespace deciders {

/**\brief Empty Node Registration Decider
 *
 * Handy when you are testing other parts of the application but not the
 * specific registration procedure
 *
 * \ingroup mrpt_graphslam_grp
 */
template<class GRAPH_t=typename mrpt::graphs::CNetworkOfPoses2DInf>
class CEmptyNRD:
	public mrpt::graphslam::deciders::CNodeRegistrationDecider<GRAPH_t>
{
	typedef typename GRAPH_t::constraint_t::type_value pose_t;
	public:
		CEmptyNRD();
		~CEmptyNRD();

		bool updateState( mrpt::obs::CActionCollectionPtr action,
				mrpt::obs::CSensoryFramePtr observations,
				mrpt::obs::CObservationPtr observation );
		pose_t getCurrentRobotPosEstimation() const;

	private:
		void registerNewNode();

};

//////////////////////////////////////////////////////////////////////////////

template<class GRAPH_t>
CEmptyNRD<GRAPH_t>::CEmptyNRD() { }
template<class GRAPH_t>
CEmptyNRD<GRAPH_t>::~CEmptyNRD() { }

template<class GRAPH_t>
bool CEmptyNRD<GRAPH_t>::updateState(
		mrpt::obs::CActionCollectionPtr action,
		mrpt::obs::CSensoryFramePtr observations,
		mrpt::obs::CObservationPtr observation )  {return false;}

template<class GRAPH_t>
void CEmptyNRD<GRAPH_t>::registerNewNode() { }

template<class GRAPH_t>
typename GRAPH_t::constraint_t::type_value
CEmptyNRD<GRAPH_t>::getCurrentRobotPosEstimation() const {return pose_t();}

} } } // end of namespaces

#endif /* end of include guard: CEMPTYNRD_H */
