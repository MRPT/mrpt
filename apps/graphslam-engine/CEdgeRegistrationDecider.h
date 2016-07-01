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

#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/graphs/CNetworkOfPoses.h>
#include <mrpt/utils/types_simple.h>
#include <mrpt/synch/CCriticalSection.h>
#include <mrpt/utils/TParameters.h>

#include "CWindowManager.h"

#include <map>
#include <string>

namespace mrpt { namespace graphslam { namespace deciders {

/**
	* Interface for implementing edge registration classes.
	*/
template<class GRAPH_t=typename mrpt::graphs::CNetworkOfPoses2DInf>
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
		 	*/
		virtual void updateDeciderState(
				mrpt::obs::CActionCollectionPtr action,
				mrpt::obs::CSensoryFramePtr observations,
				mrpt::obs::CObservationPtr observation ) = 0;
		/**
			* method for fetching the graph after the instance initialization
			*/
		virtual void setGraphPtr(GRAPH_t* graph) {}
		// set the rawlog fname - may be needed in the derived classes
		virtual void setRawlogFname(const std::string& rawlog_fname) {}
		/**
			* Method for fetching the CWindowManager pointer, responsible for
			* various parameters configuration in the CDisplayWindow
			*/
    virtual void setWindowManagerPtr(mrpt::gui::CWindowManager_t* win_manager){}
		/**
			* Fill the given map with the type of registered edges as well as
			* the corresponding number of registration of each edge.
			*/
    virtual void getEdgesStats(
    		std::map<const std::string, int>* edge_type_to_num) {};
    /**
     	* Fetch a mrpt::synch::CCriticalSection for locking the GRAPH_t resource.
     	* Handy for realising multithreading in the derived classes. Beware that
     	* prior to the decider public method call, the CCriticalSection will
     	* already be locked by the CGraphSlamEngine instance, but this isn't
     	* effective in multithreaded implementations where the decider itself
     	* has to lock the function at which the extra thread runs.
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
			* Method responsible for rendering visual objects in CDisplayWindow. For
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
    	* Method for the caller to query for possible loop closures in the
    	* last edge registration procedure.
    	*/
    virtual bool justInsertedLoopClosure() {return false;}
    /**
     	* Load the necessary for the decider parameters 
     	*/
		virtual void loadParams(const std::string& source_fname) {}
    /** 
     	* Print the problem parameters to the screen in a unified way
     	*/
		virtual void printParams() const {} 

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
		/**
			* Check if the current decider is inappropriate for the given dataset /
			* provided observatins
			*/
		virtual void checkIfInvalidDataset(
				mrpt::obs::CActionCollectionPtr action,
				mrpt::obs::CSensoryFramePtr observations,
				mrpt::obs::CObservationPtr observation ) {}
};

} } } // end of namespaces
#endif /* end of include guard: CEDGEREGISTRATIONDECIDER_H */
