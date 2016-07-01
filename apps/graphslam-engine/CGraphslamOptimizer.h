/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CGRAPHOPTIMIZER_H
#define CGRAPHOPTIMIZER_H

#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/graphs/CNetworkOfPoses.h>
#include <mrpt/synch/CCriticalSection.h>

#include "CWindowManager.h"

namespace mrpt { namespace graphslam { namespace optimizers {

/**
	* Interface for implementing graph slam optimizer classes. Interface should
	* provide a general class from which real optimizers should inherit so that
	* they abide to the necessary method calls used in the CGraphSlamEngine
	* class
	*/
template<class GRAPH_t=typename mrpt::graphs::CNetworkOfPoses2DInf>
class CGraphSlamOptimizer_t {
	public:
		typedef typename GRAPH_t::constraint_t constraint_t; // type of underlying constraints
		typedef typename GRAPH_t::constraint_t::type_value pose_t; // type of underlying poses (2D/3D)

		CGraphSlamOptimizer_t() { }
		~CGraphSlamOptimizer_t() { }

		/**
		 	* Method responsible for initially inserting visual objects in
		 	* CDisplayWindow (e.g. add an object to scene).  For the method to
		 	* have an effect user should first make a call to
		 	* CGraphSlamOptimizer_t::setCDisplayWindowPtr method.
		 	*/
    virtual void initializeVisuals() {}
		/**
		 	* Method responsible for rendering visual objects in CDisplayWindow.
		 	* For the method to have an effect user should first make a call to
		 	* CGraphSlamOptimizer_t::setCDisplayWindowPtr method.
		 	*/
    virtual void updateVisuals() {}
		/**
		 	* Generic method for fetching the incremental action/observation
		 	* readings from the calling function. Implementations of this
		 	* interface should use (part of) the specified parameters and call the
		 	* optimizeGraph function if the decision is to optimize the provided
		 	* graph
		 	*
		 	* Returns true if the optimization procedure was executed.
		 	*/
		virtual bool updateOptimizerState(
				mrpt::obs::CActionCollectionPtr action,
				mrpt::obs::CSensoryFramePtr observations,
				mrpt::obs::CObservationPtr observation ) = 0;

		/**
		 * Fetch the graph after the instance initialization
		 */
		virtual void setGraphPtr(GRAPH_t* graph) {}
		/**
		 * set the rawlog fname - may be needed in the derived classes
		 */
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
     * Fetch a mrpt::synch::CCriticalSection for locking the GRAPH_t resource.
     * Handy for realising multithreading in the derived classes. Beware that
     * prior to the optimizer public method call, the CCriticalSection will
     * already be locked, but this isn't effective in multithreaded
     * implementations where the decider itself has to lock the function at
     * which the extra thread runs.
     */
		virtual void setCriticalSetionPtr(
				mrpt::synch::CCriticalSection* graph_section) { }
    /**
     * Load the necessary for the optimizer parameters 
     */
		virtual void loadParams(const std::string& source_fname) {}
    /** 
     * Print the problem parameters to the screen in a unified way
     */
		virtual void printParams() const {}

	protected:
		/**
		 	* Method called for optimizing the underlying graph.
		 	*/
		virtual void optimizeGraph() = 0;

};

} } } // end of namespaces

#endif /* end of include guard: CGRAPHOPTIMIZER_H */
