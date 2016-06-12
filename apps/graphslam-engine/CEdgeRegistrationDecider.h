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

#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/graphs/CNetworkOfPoses.h>
#include <mrpt/utils/types_simple.h>

#include <map>
#include <string>

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
		 		 * Returns map of edge types to number of edges for each corresponding
		 		 * type.
		 		 */
				virtual void updateDeciderState(
						mrpt::obs::CActionCollectionPtr action,
						mrpt::obs::CSensoryFramePtr observations,
						mrpt::obs::CObservationPtr observation ) = 0;
				/**
				 * method for fetching the graph after the instance initialization
				 */
				virtual void setGraphPtr(GRAPH_t* graph) {}
				/**
				 * Method for fetching the CDisplayWindow3D after the instance
				 * initialization. Handy so that the node registrator may add visual
				 * information.
				 */
				virtual void setCDisplayWindowPtr(mrpt::gui::CDisplayWindow3D* win){}
				/**
				 * Fill the given map with the type of registered edges as well as
				 * the corresponding number of registration of each edge.
				 */
    		virtual void getEdgesStats(
    				std::map<const std::string, int>* edge_type_to_num) {};
				/**
				 * Method responsible for initially inserting visual objects in
				 * CDisplayWindow (e.g. add an object to scene).  For the method to
				 * have an effect user should first make a call to
				 * CEdgeRegistrationDEcider_t::setCDisplayWindowPtr method.
				 */
    		virtual void initializeVisuals() {}
				/**
				 * Method responsible for rendering visual objects in CDisplayWindow.
				 * For the method to have an effect user should first make a call to
				 * CEdgeRegistrationDEcider_t::setCDisplayWindowPtr method.
				 */
    		virtual void updateVisuals() {}

  		protected:
				/**
		 		 * Method for checking whether a new edge should be registered in the
		 		 * graph. If condition(s) for edge registration is satisfied, method
		 		 * should call the registerNewEdge method.
		 		 */
				virtual void checkRegistrationCondition(
						mrpt::utils::TNodeID from,
						mrpt::utils::TNodeID to ) {}
		 		/**
		 		 * Method for checking whether new edges should be registered in the
		 		 * graph. If condition(s) for edge registration is satisfied, method
		 		 * should call the registerNewEdge method.
		 		 */
				virtual void checkRegistrationCondition(
						const std::set<mrpt::utils::TNodeID>&) {}
				/**
		 		 * Wrapper around GRAPH_t::insertEdge method
     		 */
    		virtual void registerNewEdge(
    				const mrpt::utils::TNodeID& from, 
    				const mrpt::utils::TNodeID& to,
    				const constraint_t& rel_edge) = 0;
		};

} } } // end of namespaces
#endif /* end of include guard: CEDGEREGISTRATIONDECIDER_H */
