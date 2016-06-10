/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CEDGEREGISTRATIONDECIDER_H
#define CEDGEREGISTRATIONDECIDER_H

#include <mrpt/utils/types_simple.h>

namespace mrpt { namespace graphslam { namespace deciders {

	/**
	 * Interface for implementing edge registration classes.
	 */
	template<class GRAPH_t>
		class CEdgeRegistrationDecider_t {
  		public:
				typedef typename GRAPH_t::constraint_t constraint_t;
				typedef typename GRAPH_t::constraint_t::type_value pose_t; // type of underlying poses (2D/3D)

    		CEdgeRegistrationDecider_t() {}
    		virtual ~CEdgeRegistrationDecider_t() {};
				/**
		 		 * Generic method for fetching the incremental action/observation
		 		 * readings from the calling function. Implementations of this
		 		 * interface should use
		 		 * (part of) the specified parameters and call the
		 		 * checkRegistrationCondition to check for potential Edge registration
		 		 *
		 		 * Returns true upon successful Edge registration in the graph
		 		 */
				virtual bool updateDeciderState( 
						mrpt::obs::CActionCollectionPtr action,
						mrpt::obs::CSensoryFramePtr observations,
						mrpt::obs::CObservationPtr observation ) = 0;

  		protected:
				/**
		 		 * Methods for checking whether new edges should be registered in the
		 		 * graph. If condition(s) for edge registration is satisfied, method
		 		 * should call the registerNewEdge method.
		 		 *
		 		 * Returns true upon successful Edge registration in the graph
		 		 */
				virtual bool checkRegistrationCondition(
						mrpt::utils::TNodeID from
						mrpt::utils::TNodeID to ) {}
				virtual bool checkRegistrationCondition(std::set<mrpt::utils::TNodeID>){}
				/**
		 		 * Generic method of adding new poses to the graph. 
		 		 *
		 		 * Returns true upon successful Edge registration in the graph, false
		 		 * otherwise
     		 */
    		virtual bool registerNewEdge() = 0;
		};

} } } // end of namespaces
#endif /* end of include guard: CEDGEREGISTRATIONDECIDER_H */
