/* +---------------------------------------------------------------------------+
	 |                     Mobile Robot Programming Toolkit (MRPT)               |
	 |                          http://www.mrpt.org/                             |
	 |                                                                           |
	 | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
	 | See: http://www.mrpt.org/Authors - All rights reserved.                   |
	 | Released under BSD License. See details in http://www.mrpt.org/License    |
	 +---------------------------------------------------------------------------+ */

#ifndef CNODEREGISTRATIONDECIDER_H
#define CNODEREGISTRATIONDECIDER_H


#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/graphs/CNetworkOfPoses.h>
#include <mrpt/synch/CCriticalSection.h>
#include <mrpt/utils/TParameters.h>
#include <mrpt/utils/CTimeLogger.h>

#include <mrpt/graphslam/misc/CWindowManager.h>
#include "CRegistrationDeciderOrOptimizer.h"


namespace mrpt { namespace graphslam { namespace deciders {

/**\brief Interface for implementing node registration classes.
 *
 * CNodeRegistrationDecider provides the basic methods that have to exist in
 * every node registration decider class. For an example of inheriting from
 * this class see CFixedIntervalsNRD.
 *
 * \note As a naming convention, all the implemented node registration deciders
 * are suffixed with the NRD acronym.
 *
 * \ingroup mrpt_graphslam_grp
 */
template<class GRAPH_t=typename mrpt::graphs::CNetworkOfPoses2DInf>
class CNodeRegistrationDecider : public mrpt::graphslam::CRegistrationDeciderOrOptimizer<GRAPH_t> {
	public:
		/**\brief type of graph constraints */
		typedef typename GRAPH_t::constraint_t constraint_t;
		/**\brief type of underlying poses (2D/3D). */
		typedef typename GRAPH_t::constraint_t::type_value pose_t;

		/**\brief Default class constructor.*/
		CNodeRegistrationDecider() {}
		/**\brief Default class destructor.*/
		virtual ~CNodeRegistrationDecider() {};

		/** \return Latest estimated robot position
		 */
		virtual pose_t getCurrentRobotPosEstimation() const = 0;

		/**\brief Generic method for fetching the incremental action-observations (or
		 * observation-only) depending on the rawlog format readings from the
		 * calling function.
		 *
		 * Implementations of this interface should use (part of) the specified
		 * parameters and call the checkRegistrationCondition to check for
		 * potential node registration
		 *
		 * \return True upon successful node registration in the graph
		 */
		virtual bool updateState(
				mrpt::obs::CActionCollectionPtr action,
				mrpt::obs::CSensoryFramePtr observations,
				mrpt::obs::CObservationPtr observation ) = 0;

	protected:
		/**\brief Check whether a new node should be registered in the
		 * graph.
		 *
		 * This should be the key-method in any implementation of this
		 * interface. Should call registerNewNode method if the registration
		 * condition is satisfied.
		 *
		 * \return True upon successful node registration in the graph
		 */
		virtual bool checkRegistrationCondition() {return false;}
		/**\brief Generic method of adding new poses to the graph.
		 */
		virtual void registerNewNode() = 0;

};

} } } // end of namespaces

#endif /* end of include guard: CNODEREGISTRATIONDECIDER_H */
