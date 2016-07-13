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
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/CConfigFileBase.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/utils/types_simple.h>
#include <mrpt/utils/TColor.h>
#include <mrpt/system/threads.h>
#include <mrpt/opengl/graph_tools.h>
#include <mrpt/opengl/CDisk.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/utils/TColor.h>

#include "CGraphslamOptimizer.h"
#include "CWindowManager.h"
#include "CWindowObserver.h"

#include <iostream>
#include <string>
#include <map>
#include <cmath> // fabs function

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
				// True if optimization procedure is to run in a multithreading fashion
				bool optimization_on_second_thread;

				// optimize only for the nodes found in a certain distance from the
				// current position. Optimize for the entire graph if set to -1
				double optimization_distance;
				double offset_y_optimization_distance;
				int text_index_optimization_distance;
				mrpt::utils::TColor optimization_distance_color;
				std::string keystroke_optimization_distance;
				
				// nodeID difference for an edge to be considered loop closure
				int LC_min_nodeid_diff;

				// Map of TPairNodesID to their corresponding edge as recorded in the
				// last update of the optimizer state
				typename GRAPH_t::edges_map_t last_pair_nodes_to_edge;
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
				std::string keystroke_graph_toggle; // see Ctor for initialization
				std::string keystroke_graph_autofit; // see Ctor for initialization
 				int text_index_graph;
 				double offset_y_graph;

    };
		void getDescriptiveReport(std::string* report_str) const; 

		// Public members
		// ////////////////////////////
    OptimizationParams opt_params;
    GraphVisualizationParams viz_params;

  private:

  	// Private methods
  	// ////////////////////////////
		
		/**
		 * Feedback of the levenberg-marquardt graph optimization procedure
		 */
		static void levMarqFeedback(
				const GRAPH_t& graph,
				const size_t iter,
				const size_t max_iter,
				const double cur_sq_error );

  	/**
  	 * Optimize the given graph. Wrapper around the
  	 * graphslam::optimize_spa_levmarq method
  	 */
		void _optimizeGraph();
		/**
		 * Wrapper around _optimizeGraph which first locks the section and then
		 * calls the _optimizeGraph method. Used in multithreaded optimization
		 * \sa optimizeGraph()
		 */
  	void optimizeGraph();
  	/**
  	 * Match the previously registered edges in the graph with the current.
  	 * If there is a node difference in any new edge greater than
  	 * LC_min_nodeid_diff (see .ini parameter) then a full graph optimization
  	 * is issued. Returns true on new loop closure
  	 */
  	bool checkForLoopClosures();
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
		/**
		 * Set the camera parameters of the CDisplayWindow3D so that the whole
		 * graph is viewed in the window. Method assumes that the COpenGLinstance
		 * *is not locked* prior to this call, so make sure you have issued
		 * CDisplayWindow3D::unlockAccess3DScene() before calling this method.
		 */
		inline void fitGraphInView();

		void initOptDistanceVisualization();
		/**
		 * Update the position of the disk indicating the distance in which
		 * Levenberg-Marquardt graph optimization is executed
		 */
		inline void updateOptDistanceVisualization();
		void toggleOptDistanceVisualization();

		void getNearbyNodesOf(
		 		std::set<mrpt::utils::TNodeID> *nodes_set,
				const mrpt::utils::TNodeID& cur_nodeID,
				double distance );

		// Private members
		//////////////////////////////////////////////////////////////
		GRAPH_t* m_graph;
		mrpt::gui::CDisplayWindow3D* m_win;
		mrpt::gui::CWindowManager_t* m_win_manager;
		mrpt::gui::CWindowObserver* m_win_observer;
		mrpt::synch::CCriticalSection* m_graph_section;

		std::string m_rawlog_fname;

		bool m_first_time_call;
		bool m_initialized_visuals;
		bool m_has_read_config;
		bool m_just_inserted_loop_closure;
		bool registered_new_node;
		bool m_autozoom_active;

		// start optimizing the graph after a certain number of nodes has been
		// added (when m_graph->nodeCount() > m_last_total_num_of_nodes)
		size_t m_last_total_num_of_nodes;

		// Use second thread for graph optimization
		mrpt::system::TThreadHandle m_thread_optimize;

		// logger
		mrpt::utils::COutputLogger_t m_out_logger;
		mrpt::utils::CTimeLogger m_time_logger;

};

} } } // end of namespaces

#include "CLevMarqGSO_impl.h"

#endif /* end of include guard: CLEVMARQGSO_H */
