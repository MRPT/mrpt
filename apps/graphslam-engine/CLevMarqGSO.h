/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CLEVMARQGSO_H
#define CLEVMARQGSO_H

#include <mrpt/graphslam.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils/CConfigFileBase.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/utils/types_simple.h>
#include <mrpt/utils/TColor.h>
#include <mrpt/system/threads.h>
#include <mrpt/opengl/graph_tools.h>

#include "CGraphslamOptimizer.h"
#include "CWindowManager.h"
#include "CWindowObserver.h"

#include <iostream>
#include <string>
#include <map>

namespace mrpt { namespace graphslam { namespace optimizers {

/**
 * Levenberg-Marquardt non-linear graph slam optimization scheme
 */

template<class GRAPH_t=typename mrpt::graphs::CNetworkOfPoses2DInf>
class CLevMarqGSO_t:
	public mrpt::graphslam::optimizers::CGraphSlamOptimizer_t<GRAPH_t>
{
  public:
		// Public methods
		//////////////////////////////////////////////////////////////

		typedef typename GRAPH_t::constraint_t constraint_t;
		typedef typename GRAPH_t::constraint_t::type_value pose_t; // type of underlying poses (2D/3D)
		typedef mrpt::math::CMatrixFixedNumeric<double,
						constraint_t::state_length,
						constraint_t::state_length> InfMat;

    CLevMarqGSO_t();
    ~CLevMarqGSO_t();
    void initCLevMarqGSO_t();

		bool updateOptimizerState( mrpt::obs::CActionCollectionPtr action,
				mrpt::obs::CSensoryFramePtr observations,
				mrpt::obs::CObservationPtr observation );

		void setGraphPtr(GRAPH_t* graph);
		void setRawlogFname(const std::string& rawlog_fname);
    void setWindowManagerPtr(mrpt::gui::CWindowManager_t* win_manager);
		void setCriticalSectionPtr(mrpt::synch::CCriticalSection* graph_section);
    void initializeVisuals();
    void updateVisuals();
		/**
		 * Get a list of the window events that happened since the last call.
		 */
		void notifyOfWindowEvents(const std::map<std::string, bool> events_occurred); 
		// struct for holding the optimization-related variables in a compact form
    struct OptimizationParams: public mrpt::utils::CLoadableOptions {
    	public:
    		OptimizationParams();
    		~OptimizationParams();

    		void loadFromConfigFile(
    				const mrpt::utils::CConfigFileBase &source,
    				const std::string &section);
				void 	dumpToTextStream(mrpt::utils::CStream &out) const;

				TParametersDouble cfg;
				bool optimization_on_second_thread;
    };

		void loadParams(const std::string& source_fname);
		void printParams() const; 

		// struct for holding the graph visualization-related variables in a
		// compact form
    struct GraphVisualizationParams: public mrpt::utils::CLoadableOptions {
    	public:
    		GraphVisualizationParams();
    		~GraphVisualizationParams();

    		void loadFromConfigFile(
    				const mrpt::utils::CConfigFileBase &source,
    				const std::string &section);
				void 	dumpToTextStream(mrpt::utils::CStream &out) const;

				TParametersDouble cfg;
				bool visualize_optimized_graph;
				// textMessage parameters
				std::string keystroke_graph;
 				int text_index_graph;
 				double offset_y_graph;

    };

		// Public members
		// ////////////////////////////
    OptimizationParams opt_params;
    GraphVisualizationParams viz_params;

  private:

		static void levMarqFeedback(
				const GRAPH_t& graph,
				const size_t iter,
				const size_t max_iter,
				const double cur_sq_error );

  	// Private methods
		void optimizeGraph();
		/**
		 * Wrapper around optimizeGraph which first locks the section and then
		 * calls the optimizeGraph method
		 * \sa optimizegraph()
		 */
  	void optimizeGraph_2ndThread();
		void initGraphVisualization();
		/**
		 * Called internally for updating the vizualization scene for the graph
		 * building procedure
		 */
		inline void updateGraphVisualization();
		/**
		 * togle the graph visualization on and off
		 */
		void toggleGraphVisualization();

		// Private members
		//////////////////////////////////////////////////////////////
		GRAPH_t* m_graph;
		mrpt::gui::CDisplayWindow3D* m_win;
		mrpt::gui::CWindowManager_t* m_win_manager;
		mrpt::gui::CWindowObserver* m_win_observer;
		mrpt::synch::CCriticalSection* m_graph_section;

		std::string m_rawlog_fname;

		bool m_initialized_visuals;
		bool m_just_inserted_loop_closure;
		bool registered_new_node;

		// start optimizing the graph after a certain number of nodes has been
		// added (when m_graph->nodeCount() > m_last_total_num_of_nodes)
		size_t m_last_total_num_of_nodes;

		// Use second thread for graph optimization
		mrpt::system::TThreadHandle m_thread_optimize;

};

} } } // end of namespaces

#include "CLevMarqGSO_impl.h"

#endif /* end of include guard: CLEVMARQGSO_H */
