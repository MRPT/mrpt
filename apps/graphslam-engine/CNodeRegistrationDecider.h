/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
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

namespace mrpt { namespace graphslam { namespace deciders {

	/**
	 * Interface for implementing node registration classes.
	 */
	template<
		  class GRAPH_t=typename mrpt::graphs::CNetworkOfPoses2DInf>
		class CNodeRegistrationDecider_t {
  		public:
				typedef typename GRAPH_t::constraint_t constraint_t;
				typedef typename GRAPH_t::constraint_t::type_value pose_t; // type of underlying poses (2D/3D)

    		CNodeRegistrationDecider_t() {}
    		virtual ~CNodeRegistrationDecider_t() {};

				/**
		 		 * Generic method for fetching the incremental action/observation
		 		 * readings from the calling function. Implementations of this
		 		 * interface should use (part of) the specified parameters and call the
		 		 * checkRegistrationCondition to check for potential node registration
		 		 *
		 		 * Returns true upon successful node registration in the graph
		 		 */
				virtual bool updateDeciderState(
						mrpt::obs::CActionCollectionPtr action,
						mrpt::obs::CSensoryFramePtr observations,
						mrpt::obs::CObservationPtr observation ) = 0;

				// method for fetching the graph after the instance initialization
				virtual void setGraphPtr(GRAPH_t* graph) {}
				// set the rawlog fname - may be needed in the derived classes
				virtual void setRawlogFname(const std::string& rawlog_fname) {}
				/**
				 * method for fetching the CDisplayWindow3D after the instance
				 * initialization. Handy so that the node registrator can add visual
				 * information.
				 */
				virtual void setCDisplayWindowPtr(mrpt::gui::CDisplayWindow3D* win) {}
				/**
				 * Method for fetching the CWindowManager pointer, responsible for
				 * various parameters configuration in the CDisplayWindow
				 */
    		virtual void setWindowManagerPtr(mrpt::gui::CWindowManager_t* win_manager){}
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
				 * CNodeRegistrationDEcider_t::setCDisplayWindowPtr method.
				 */
    		virtual void updateVisuals() {}

  		protected:
				/**
		 		 * Method for checking whether a new node should be registered in the
		 		 * graph. This should be the key-method in any implementation of this
		 		 * interface. Should call registerNewNode method if the registration
		 		 * condition is satisfied.
		 		 *
		 		 * Returns true upon successful node registration in the graph
		 		 */
				virtual bool checkRegistrationCondition() {return false;}
				/**
		 		 * Generic method of adding new poses to the graph.
     		 */
    		virtual void registerNewNode() = 0;
				/**
			 	 * Check if the current decider is inappropriate for the given dataset /
			 	 * provided observatins
			 	 */
				void checkIfInvalidDataset(
						mrpt::obs::CActionCollectionPtr action,
						mrpt::obs::CSensoryFramePtr observations,
						mrpt::obs::CObservationPtr observation ) {}

		};

} } } // end of namespaces

#endif /* end of include guard: CNODEREGISTRATIONDECIDER_H */
