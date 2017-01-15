/* +---------------------------------------------------------------------------+
	 |                     Mobile Robot Programming Toolkit (MRPT)               |
	 |                          http://www.mrpt.org/                             |
	 |                                                                           |
	 | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
	 | See: http://www.mrpt.org/Authors - All rights reserved.                   |
	 | Released under BSD License. See details in http://www.mrpt.org/License    |
	 +---------------------------------------------------------------------------+ */

#ifndef CLEVMARQGSO_H
#define CLEVMARQGSO_H

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

#include <mrpt/graphslam/levmarq.h>
#include <mrpt/graphslam/interfaces/CGraphSlamOptimizer.h>

#include <iostream>
#include <string>
#include <map>
#include <cmath> // fabs function

namespace mrpt { namespace graphslam { namespace optimizers {

/**\brief Levenberg-Marquardt non-linear graph slam optimization scheme.
 *
 * ## Description
 *
 * Current decider optimizes the graph according to the
 * graphslam::optimize_spa_levmarq method. Refer to the latter for more
 * details on the implementation.
 *
 * ### .ini Configuration Parameters
 *
 * \htmlinclude graphslam-engine_config_params_preamble.txt
 *
 * - \b class_verbosity
 *   + \a Section       : OptimizerParameters
 *   + \a Default value : 1 (LVL_INFO)
 *   + \a Required      : FALSE
 *
 * - \b optimization_on_second_thread
 *   + \a Section       : OptimizerParameters
 *   + \a Default value :  FALSE
 *   + \a Required      : FALSE
 *   + \a Description   : Specify whether to use a second thread to optimize
 *   the graph.
 *
 * - \b LC_min_nodeid_diff
 *  + \a Section       : GeneralConfiguration
 *  + \a Default value : 30
 *  + \a Required      : FALSE
 *  + \a Description   : Minimum NodeID difference for an edge to be considered
 *  a loop closure.
 *
 * - \b optimization_distance
 *  + \a Section       : OptimizerParameters
 *  + \a Default value : 5
 *  + \a Required      : FALSE
 *  + \a Description   : Positions of the nodes within the specified distance from the current
 *  graph node are optimized according to the corresponding constraints between
 *  them
 *  
 * - \b verbose
 *  + \a Section       : OptimizerParameters
 *  + \a Default value : FALSE
 *  + \a Required      : FALSE
 *  + \a Description   : Refers to the Levenberg-Marquardt optimization.
 *  procedure
 *
 * - \b profiler
 *  + \a Section       : OptimizerParameters
 *  + \a Default value : FALSE
 *  + \a Required      : FALSE
 *  + \a Description   : Refers to the Levenberg-Marquardt optimization.
 *  procedure
 *
 * - \b max_iterations
 *  + \a Section       : OptimizerParameters
 *  + \a Default value : 100
 *  + \a Required      : FALSE
 *  + \a Description   : Refers to the Levenberg-Marquardt optimization. Sets
 *  the maximum number of iterations of the optimization scheme.
 *
 * - \b scale_hessian
 *  + \a Section       : OptimizerParameters
 *  + \a Default value : 0.2
 *  + \a Required      : FALSE
 *  + \a Description   : Refers to the Levenberg-Marquardt optimization.
 *
 * - \b tau
 *  + \a Section       : OptimizerParameters
 *  + \a Default value : 1e-3
 *  + \a Required      : FALSE
 *  + \a Description   : Refers to the Levenberg-Marquardt optimization.
 *
 *  \note For a detailed description of the optimization parameters of the
 *  Levenberg-Marquardt scheme, refer to
 *  http://reference.mrpt.org/devel/group__mrpt__graphslam__grp.html#ga022f4a70be5ec7c432f46374e4bb9d66 
 *
 *  \note For a detailed description of the graph visualization parameters
 *  refer to
 *  http://reference.mrpt.org/devel/group__mrpt__opengl__grp.html#ga30efc9f6fcb49801e989d174e0f65a61

 *
 * \ingroup mrpt_graphslam_grp
 */
template<class GRAPH_t=typename mrpt::graphs::CNetworkOfPoses2DInf>
class CLevMarqGSO:
	public mrpt::graphslam::optimizers::CGraphSlamOptimizer<GRAPH_t>
{
	public:
		// Public methods
		//////////////////////////////////////////////////////////////

		typedef typename GRAPH_t::constraint_t constraint_t;
		typedef typename GRAPH_t::constraint_t::type_value pose_t; // type of underlying poses (2D/3D)
		typedef mrpt::math::CMatrixFixedNumeric<double,
						constraint_t::state_length,
						constraint_t::state_length> InfMat;

		CLevMarqGSO();
		~CLevMarqGSO();
		void initCLevMarqGSO();

		bool updateState( mrpt::obs::CActionCollectionPtr action,
				mrpt::obs::CSensoryFramePtr observations,
				mrpt::obs::CObservationPtr observation );

		void setGraphPtr(GRAPH_t* graph);
		void setWindowManagerPtr(mrpt::graphslam::CWindowManager* win_manager);
		void setCriticalSectionPtr(mrpt::synch::CCriticalSection* graph_section);
		void initializeVisuals();
		void updateVisuals();
		/**\brief Get a list of the window events that happened since the last
		 * call.
		 */
		void notifyOfWindowEvents(const std::map<std::string, bool>& events_occurred);
		/**\brief Struct for holding the optimization-related variables in a
		 * compact form
		 */
		struct OptimizationParams: public mrpt::utils::CLoadableOptions {
			public:
				OptimizationParams();
				~OptimizationParams();

				void loadFromConfigFile(
						const mrpt::utils::CConfigFileBase &source,
						const std::string &section);
				void 	dumpToTextStream(mrpt::utils::CStream &out) const;

				mrpt::utils::TParametersDouble cfg;
				// True if optimization procedure is to run in a multithreading fashion
				bool optimization_on_second_thread;

				/**\brief optimize only for the nodes found in a certain distance from
				 * the current position. Optimize for the entire graph if set to -1
				 */
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

		/**\brief struct for holding the graph visualization-related variables in a
		 * compact form
		 */
		struct GraphVisualizationParams: public mrpt::utils::CLoadableOptions {
			public:
				GraphVisualizationParams();
				~GraphVisualizationParams();

				void loadFromConfigFile(
						const mrpt::utils::CConfigFileBase &source,
						const std::string &section);
				void dumpToTextStream(mrpt::utils::CStream &out) const;

				mrpt::utils::TParametersDouble cfg;
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
		OptimizationParams opt_params; /**<Parameters relevant to the optimizatio nfo the graph. */
		GraphVisualizationParams viz_params; /**<Parameters relevant to the visualization of the graph. */

	private:

		// Private methods
		// ////////////////////////////

		/**\brief Feedback of the Levenberg-Marquardt graph optimization procedure.
		 */
		static void levMarqFeedback(
				const GRAPH_t& graph,
				const size_t iter,
				const size_t max_iter,
				const double cur_sq_error );

		/**\brief Optimize the given graph.
		 *
		 * Wrapper around the graphslam::optimize_spa_levmarq method
		 * \sa optimize_spa_levmarq, optimizeGraph
		 */
		void _optimizeGraph();
		/** \brief Wrapper around _optimizeGraph which first locks the section and then
		 * calls the _optimizeGraph method.
		 *
		 * Used in multithreaded optimization
		 * \sa _optimizeGraph()
		 */
		void optimizeGraph();
		/**\brief Checks if a loop closure edge was added in the graph.
		 *
		 * Match the previously registered edges in the graph with the current. If
		 * there is a node difference *in any new edge* greater than
		 * \b LC_min_nodeid_diff (see .ini parameter) then a full graph optimization
		 * is issued.
		 *
		 * \return True on new loop closure
		 */
		bool checkForLoopClosures();
		void initGraphVisualization();
		/**\brief Called internally for updating the visualization scene for the graph
		 * building procedure
		 */
		inline void updateGraphVisualization();
		/**\brief Toggle the graph visualization on and off.
		 */
		void toggleGraphVisualization();
		/**\brief Set the camera parameters of the CDisplayWindow3D so that the whole
		 * graph is viewed in the window.
		 *
		 * \warning Method assumes that the COpenGLinstance *is not locked* prior to this
		 * call, so make sure you have issued
		 * CDisplayWindow3D::unlockAccess3DScene() before calling this method.
		 */
		inline void fitGraphInView();

		void initOptDistanceVisualization();
		/**\brief Update the position of the disk indicating the distance in which
		 * Levenberg-Marquardt graph optimization is executed
		 */
		inline void updateOptDistanceVisualization();
		/**\brief toggle the optimization distance disk on and off
		 */
		void toggleOptDistanceVisualization();
		/**\brief Get a list of the nodeIDs whose position is within a certain
		 * distance to the specified nodeID
		 */
		void getNearbyNodesOf(
				std::set<mrpt::utils::TNodeID> *nodes_set,
				const mrpt::utils::TNodeID& cur_nodeID,
				double distance );

		// Private members
		//////////////////////////////////////////////////////////////
		GRAPH_t* m_graph; /**<\brief Pointer to the graph under construction */
		mrpt::gui::CDisplayWindow3D* m_win;
		mrpt::graphslam::CWindowManager* m_win_manager;
		mrpt::graphslam::CWindowObserver* m_win_observer;
		mrpt::synch::CCriticalSection* m_graph_section;

		bool m_first_time_call;
		bool m_initialized_visuals;
		bool m_has_read_config;
		bool registered_new_node;
		bool m_autozoom_active;

		// start optimizing the graph after a certain number of nodes has been
		// added (when m_graph->nodeCount() > m_last_total_num_of_nodes)
		size_t m_last_total_num_of_nodes;

		// Use second thread for graph optimization
		mrpt::system::TThreadHandle m_thread_optimize;
		mrpt::utils::CTimeLogger m_time_logger; /**<Time logger instance */
};

} } } // end of namespaces

#include "CLevMarqGSO_impl.h"

#endif /* end of include guard: CLEVMARQGSO_H */
