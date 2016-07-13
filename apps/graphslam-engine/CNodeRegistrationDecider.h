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
#include <mrpt/synch/CCriticalSection.h>
#include <mrpt/utils/TParameters.h>
#include <mrpt/utils/CTimeLogger.h>

#include "CWindowManager.h"


namespace mrpt { namespace graphslam { namespace deciders {

/**
	* Interface for implementing node registration classes.
	*/
template<class GRAPH_t=typename mrpt::graphs::CNetworkOfPoses2DInf>
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
		 	* Method for fetching the CWindowManager pointer, responsible for
		 	* various parameters configuration in the CDisplayWindow
		 	*/
    virtual void setWindowManagerPtr(mrpt::gui::CWindowManager_t* win_manager){}
    /**
     * Fetch a mrpt::synch::CCriticalSection for locking the GRAPH_t resource.
     * Handy for realising multithreading in the derived classes. Beware that
     * prior to the optimizer public method call, the CCriticalSection will
     * already be locked, but this isn't effective in multithreaded
     * implementations where the decider itself has to lock the function at
     * which the extra thread runs.
     */
		virtual void setCriticalSectionPtr(
				mrpt::synch::CCriticalSection* graph_section) { }
		/**
		 	* Method responsible for initially inserting visual objects in
		 	* CDisplayWindow (e.g. add an object to scene).  For the method to
		 	* have an effect user should first initialize the m_win variable
		 	* \sa setWindowManagerPtr
		 	*/
    virtual void initializeVisuals() {}
		/**
		 	* Method responsible for rendering visual objects in CDisplayWindow.For
		 	* the method to have an effect user should first initialize the m_win
		 	* variable 
		 	* \sa setWindowManagerPtr
		 	*/
    virtual void updateVisuals() {}
		/**
		 * Get a list of the window events that happened since the last call.
		 * Method in derived  classes is automatically called from the
		 * CGraphSlamEngine_t instance. Optimizer should just fetch the parameters
		 * that it is interested in.
		 */
		virtual void notifyOfWindowEvents(
				const std::map<std::string, bool>& events_occurred) { }
    /**
     * Load the necessary for the decider parameters 
     */
		virtual void loadParams(const std::string& source_fname) {}
    /** 
     * Print the problem parameters to the screen in a unified way
     */
		virtual void printParams() const {} 
		/**
		 * Fill the provided string with a detailed report of the class state
		 * including:
		 * - Timing of important methods
		 * - Properties fo class at the current time
		 * - Logging of commands until current time
		 */
		virtual void getDescriptiveReport(std::string* report_str) const {}

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
