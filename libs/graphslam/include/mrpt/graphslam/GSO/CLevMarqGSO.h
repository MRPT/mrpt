/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/config/CLoadableOptions.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/img/TColor.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>

#include <mrpt/graphslam/levmarq.h>
#include <mrpt/graphslam/interfaces/CGraphSlamOptimizer.h>

#include <iostream>
#include <string>
#include <map>
#include <cmath>

namespace mrpt::graphslam::optimizers
{
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
 *   + \a Default value : 1 (mrpt::system::LVL_INFO)
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
 *  + \a Description   : Positions of the nodes within the specified distance
 from the current
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
 *
 http://reference.mrpt.org/devel/group__mrpt__graphslam__grp.html#ga022f4a70be5ec7c432f46374e4bb9d66
 *
 *  \note For a detailed description of the graph visualization parameters
 *  refer to
 *
 http://reference.mrpt.org/devel/group__mrpt__opengl__grp.html#ga30efc9f6fcb49801e989d174e0f65a61

 *
 * \ingroup mrpt_graphslam_grp
 */
template <class GRAPH_T = typename mrpt::graphs::CNetworkOfPoses2DInf>
class CLevMarqGSO
	: public mrpt::graphslam::optimizers::CGraphSlamOptimizer<GRAPH_T>
{
   public:
	// Public methods
	//////////////////////////////////////////////////////////////
	/**\brief Handy typedefs */
	/**\{*/
	using constraint_t = typename GRAPH_T::constraint_t;
	/** type of underlying poses (2D/3D)*/
	using pose_t = typename GRAPH_T::constraint_t::type_value;
	using InfMat = mrpt::math::CMatrixFixedNumeric<
		double, constraint_t::state_length, constraint_t::state_length>;
	using grandpa = mrpt::graphslam::CRegistrationDeciderOrOptimizer<GRAPH_T>;
	using parent = mrpt::graphslam::optimizers::CGraphSlamOptimizer<GRAPH_T>;
	/**\}*/

	CLevMarqGSO();
	~CLevMarqGSO() override = default;

	bool updateState(
		mrpt::obs::CActionCollection::Ptr action,
		mrpt::obs::CSensoryFrame::Ptr observations,
		mrpt::obs::CObservation::Ptr observation) override;

	void initializeVisuals() override;
	void updateVisuals() override;
	/**\brief Get a list of the window events that happened since the last
	 * call.
	 */
	void notifyOfWindowEvents(
		const std::map<std::string, bool>& events_occurred) override;
	/**\brief Struct for holding the optimization-related variables in a
	 * compact form
	 */
	struct OptimizationParams : public mrpt::config::CLoadableOptions
	{
	   public:
		OptimizationParams();
		~OptimizationParams() override;

		void loadFromConfigFile(
			const mrpt::config::CConfigFileBase& source,
			const std::string& section) override;
		void dumpToTextStream(std::ostream& out) const override;

		mrpt::system::TParametersDouble cfg;
		// True if optimization procedure is to run in a multithreading fashion
		bool optimization_on_second_thread;

		/**\brief optimize only for the nodes found in a certain distance from
		 * the current position. Optimize for the entire graph if set to1
		 */
		double optimization_distance;
		double offset_y_optimization_distance;
		int text_index_optimization_distance;
		mrpt::img::TColor optimization_distance_color;
		/**\brief Keystroke to toggle the optimization distance on/off */
		std::string keystroke_optimization_distance;
		/**\brief Keystroke to manually trigger a full graph optimization */
		std::string keystroke_optimize_graph;

		// nodeID difference for an edge to be considered loop closure
		int LC_min_nodeid_diff;

		// Map of TPairNodesID to their corresponding edge as recorded in the
		// last update of the optimizer state
		typename GRAPH_T::edges_map_t last_pair_nodes_to_edge;
	};

	/**\brief struct for holding the graph visualization-related variables in a
	 * compact form
	 */
	struct GraphVisualizationParams : public mrpt::config::CLoadableOptions
	{
	   public:
		GraphVisualizationParams();
		~GraphVisualizationParams() override;

		void loadFromConfigFile(
			const mrpt::config::CConfigFileBase& source,
			const std::string& section) override;
		void dumpToTextStream(std::ostream& out) const override;

		mrpt::system::TParametersDouble cfg;
		bool visualize_optimized_graph;
		// textMessage parameters
		std::string keystroke_graph_toggle;  // see Ctor for initialization
		std::string keystroke_graph_autofit;  // see Ctor for initialization
		int text_index_graph;
		double offset_y_graph;
	};

	void loadParams(const std::string& source_fname) override;
	void printParams() const override;
	void getDescriptiveReport(std::string* report_str) const override;

	bool justFullyOptimizedGraph() const override;

	/** Parameters relevant to the optimizatio nfo the graph. */
	OptimizationParams opt_params;
	/** Parameters relevant to the visualization of the graph. */
	GraphVisualizationParams viz_params;

   protected:
	/**\brief Feedback of the Levenberg-Marquardt graph optimization procedure.
	 *
	 */
	static void levMarqFeedback(
		const GRAPH_T& graph, const size_t iter, const size_t max_iter,
		const double cur_sq_error);

	/**\brief Optimize the given graph.
	 *
	 * Wrapper around the graphslam::optimize_spa_levmarq method
	 * \sa optimize_spa_levmarq, optimizeGraph
	 *
	 * \param[in] full_update Impose that method optimizes the whole graph
	 *
	 */
	void _optimizeGraph(bool is_full_update = false);
	/** \brief Wrapper around _optimizeGraph which first locks the section and
	 * then calls the _optimizeGraph method.
	 *
	 * Used in multithreaded optimization
	 * \sa _optimizeGraph()
	 */
	void optimizeGraph() override;
	/**\brief Check if a loop closure edge was added in the graph.
	 *
	 * Match the previously registered edges in the graph with the current. If
	 * there is a node difference *in any new edge* greater than
	 * \b LC_min_nodeid_diff (see .ini parameter) then new constraint is
	 * considered a Loop Closure
	 *
	 * \return True if \b any of the newly added edges is considered a loop
	 * closure
	 */
	bool checkForLoopClosures();
	/**\brief Decide whether to issue a full graph optimization
	 *
	 * In case N consecutive full optimizations have been issued, skip some of
	 * the next as they slow down the overall execution and they don't reduce
	 * the overall error
	 *
	 * \return True for issuing a full graph optimization, False otherwise
	 */
	bool checkForFullOptimization();
	/**\brief Initialize objects relateed to the Graph Visualization
	 */
	void initGraphVisualization();
	/**\brief Called internally for updating the visualization scene for the
	 * graph
	 * building procedure
	 */
	inline void updateGraphVisualization();
	/**\brief Toggle the graph visualization on and off.
	 */
	void toggleGraphVisualization();
	/**\brief Set the camera parameters of the CDisplayWindow3D so that the
	 * whole
	 * graph is viewed in the window.
	 *
	 * \warning Method assumes that the COpenGLinstance *is not locked* prior to
	 * this
	 * call, so make sure you have issued
	 * CDisplayWindow3D::unlockAccess3DScene() before calling this method.
	 */
	inline void fitGraphInView();

	/**\brief Initialize the Disk/Sphere used for visualizing the optimization
	 * distance.
	 */
	/**\{*/
	void initOptDistanceVisualization();
	/**\brief Setup the corresponding Disk/Sphere instance.
	 *
	 * Method overloads are used to overcome the C++ specialization
	 * restrictions
	 *
	 * \return Disk/Sphere instance for 2D/3D SLAM respectively
	 */
	/**\{*/
	mrpt::opengl::CRenderizable::Ptr initOptDistanceVisualizationInternal(
		const mrpt::poses::CPose2D& p_unused);
	mrpt::opengl::CRenderizable::Ptr initOptDistanceVisualizationInternal(
		const mrpt::poses::CPose3D& p_unused);
	/**\}*/

	/**\}*/

	/**\brief Update the position of the disk indicating the distance in which
	 * Levenberg-Marquardt graph optimization is executed
	 */
	inline void updateOptDistanceVisualization();
	/**\brief toggle the optimization distance object on and off
	 */
	void toggleOptDistanceVisualization();
	/**\brief Get a list of the nodeIDs whose position is within a certain
	 * distance to the specified nodeID
	 */
	void getNearbyNodesOf(
		std::set<mrpt::graphs::TNodeID>* nodes_set,
		const mrpt::graphs::TNodeID& cur_nodeID, double distance);

	// protected members
	//////////////////////////////////////////////////////////////

	bool m_first_time_call{false};
	bool m_has_read_config{false};
	bool registered_new_node;
	bool m_autozoom_active{true};

	// start optimizing the graph after a certain number of nodes has been
	// added (when m_graph->nodeCount() > m_last_total_num_of_nodes)
	size_t m_last_total_num_of_nodes{5};

	// Use second thread for graph optimization
	std::thread m_thread_optimize;

	/**\brief Enumeration that defines the behaviors towards using or ignoring a
	 * newly added loop closure to fully optimize the graph
	 */
	enum class OptimizationPolicy
	{
		IgnoreLoopClosures = 0,
		UseLoopClosures,

		Count
	};
	OptimizationPolicy m_optimization_policy;
	/**\name Smart Full-Optimization Command
	 *
	 * Instead of issuing a full optimization every time a loop closure is
	 * detected, ignore current loop closure when enough consecutive loop
	 * closures have already been utilised.
	 * This avoids the added computational cost that is needed for optimizing
	 * the graph without reducing the accuracy of the overall operation
	 */
	/**\{*/

	/**\brief Number of maximum cosecutive loop closures that are allowed to be
	 * issued.
	 *
	 * \sa m_curr_used_consec_lcs, m_max_ignored_consec_lcs
	 */
	size_t m_max_used_consec_lcs;
	/**\brief Number of consecutive loop closures that are currently registered
	 *
	 * \sa m_max_used_consec_lcs
	 */
	size_t m_curr_used_consec_lcs{0};
	/**\brief Number of consecutive loop closures to ignore after \b
	 * m_max_used_consec_lcs have already been issued.
	 *
	 * \sa m_curr_ignored_consec_lcs, m_max_used_consec_lcs
	 */
	size_t m_max_ignored_consec_lcs;
	/**\brief Consecutive Loop Closures that have currently been ignored
	 *
	 * \sa m_max_ignored_consec_lcs
	 */
	size_t m_curr_ignored_consec_lcs{0};

	/**\}*/

	/**\brief Indicates whether a full graph optimization was just issued.
	 */
	bool m_just_fully_optimized_graph{false};

	/**\brief Minimum number of nodes before we try optimizing the graph */
	size_t m_min_nodes_for_optimization{3};
};
}  // namespace mrpt::graphslam::optimizers
#include "CLevMarqGSO_impl.h"
