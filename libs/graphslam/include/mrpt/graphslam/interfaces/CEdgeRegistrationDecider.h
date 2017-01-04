/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CEDGEREGISTRATIONDECIDER_H
#define CEDGEREGISTRATIONDECIDER_H

#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/graphs/CNetworkOfPoses.h>
#include <mrpt/utils/types_simple.h>
#include <mrpt/synch/CCriticalSection.h>
#include <mrpt/utils/TParameters.h>

#include <mrpt/graphslam/misc/CWindowManager.h>
#include "CRegistrationDeciderOrOptimizer.h"

#include <map>
#include <string>

namespace mrpt { namespace graphslam { namespace deciders {

/** \brief Interface for implementing edge registration classes.
 *
 * CEdgeRegistrationDecider provides the basic methods that have to exist in
 * every edge registration decider class. For an example of inheriting from
 * this class see CICPCriteriaERD.
 *
 * \note As a naming convention, all the implemented edge registration deciders
 * are suffixed with the ERD acronym.
 *
 * \ingroup mrpt_graphslam_grp
 */
template<class GRAPH_t=typename mrpt::graphs::CNetworkOfPoses2DInf>
class CEdgeRegistrationDecider : public mrpt::graphslam::CRegistrationDeciderOrOptimizer<GRAPH_t> {
  public:
		/**\brief type of graph constraints */
		typedef typename GRAPH_t::constraint_t constraint_t;
		/**\brief type of underlying poses (2D/3D). */
		typedef typename GRAPH_t::constraint_t::type_value pose_t;


		/**\brief Default class constructor.*/
    CEdgeRegistrationDecider() {}
		/**\brief Default class destructor.*/
    virtual ~CEdgeRegistrationDecider() {};
		/**\brief Generic method for fetching the incremental action/observation
		 * readings from the calling function. 
		 *
		 * Implementations of this interface should use (part of) the specified
		 * parameters and call the checkRegistrationCondition to check for
		 * potential Edge registration
		 */
		virtual bool updateState(
				mrpt::obs::CActionCollectionPtr action,
				mrpt::obs::CSensoryFramePtr observations,
				mrpt::obs::CObservationPtr observation ) = 0;
		/**\brief Fill the given map with the type of registered edges as well as
		 * the corresponding number of registration of each edge.
		 */
    virtual void getEdgesStats(
    		std::map<std::string, int>* edge_type_to_num) const {};
    /**\brief Used by the caller to query for possible loop closures in the
     * last edge registration procedure.
     */
    virtual bool justInsertedLoopClosure() const {return false;}

  protected:
  	/**\name Registration criteria checks
		 *\brief Check whether a new edge should be registered in the
		 * graph. 
		 *
		 * If condition(s) for edge registration is satisfied, method should call
		 * the registerNewEdge method.
		 */
		/**\{*/
		virtual void checkRegistrationCondition(
				mrpt::utils::TNodeID from,
				mrpt::utils::TNodeID to ) {}
		virtual void checkRegistrationCondition(
				const std::set<mrpt::utils::TNodeID>&) {}
		/**\}*/

		/**\brief Wrapper around GRAPH_t::insertEdge method
     */
    virtual void registerNewEdge(
    		const mrpt::utils::TNodeID& from,
    		const mrpt::utils::TNodeID& to,
    		const constraint_t& rel_edge) = 0;
};

} } } // end of namespaces
#endif /* end of include guard: CEDGEREGISTRATIONDECIDER_H */
