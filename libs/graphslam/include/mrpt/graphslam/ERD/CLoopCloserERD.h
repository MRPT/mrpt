/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/math/CMatrix.h>
#include <mrpt/config/CLoadableOptions.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/img/TColor.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/slam/CIncrementalMapPartitioner.h>
#include <mrpt/slam/CICP.h>

#include <mrpt/graphslam/interfaces/CRangeScanEdgeRegistrationDecider.h>
#include <mrpt/graphslam/misc/TSlidingWindow.h>
#include <mrpt/graphslam/misc/TUncertaintyPath.h>
#include <mrpt/graphslam/misc/TNodeProps.h>
#include <mrpt/graphs/THypothesis.h>
#include <mrpt/graphs/CHypothesisNotFoundException.h>

#include <map>
#include <vector>
#include <string>
#include <set>
#include <utility>

namespace mrpt::graphslam::deciders
{
/**\brief Edge Registration Decider scheme specialized in Loop Closing.
 *
 * ## Description
 *
 * Current decider is implemented based on the following papers:
 *
 * - [1] <a
 *
href="http://ieeexplore.ieee.org/xpl/login.jsp?tp=&arnumber=1641810&url=http%3A%2F%2Fieeexplore.ieee.org%2Fxpls%2Fabs_all.jsp%3Farnumber%3D1641810">Consistent
 * Observation Grouping for Generating Metric-Topological Maps that Improves
 * Robot Localization</a> - J. Blanco, J. Gonzalez, J. Antonio Fernandez
 * Madrigal, 2006
 * - [2] <a
href="https://april.eecs.umich.edu/pdfs/olson2009ras.pdf">Recognizing
 * places using spectrally clustered local matches</a> - E. Olson, 2009
 *
 * ### Specifications
 *
 * - Map type: 2D
 * - MRPT rawlog format: #1, #2
 * - Graph Type: CPosePDFGaussianInf
 * - Observations: CObservation2DRangeScan
 * - Edge Registration Strategy: ICP Scan-matching between nearby nodes
 * - Loop Registration Strategy: Pair-wise Consistency of ICP Edges
 *
 * ### Loop Closing Strategy
 *
 * The Loop closure registration strategy is described below:
 *
 * - We split the graph under-construction into groups of nodes. The groups are
 *   formatted based on the observations gathered in each node position. The
 *   actual split between the groups is decided by the minimum normalized Cut
 *   (minNcut) as described in [1].
 * - Having assembled the groups of nodes, we find the groups that might
 *   contain loop closure edges (these groups contain successive nodes with
large
 *   difference in their IDs). These groups are then split into two subgroups
 *   A, B, with the former containing the lower NodeIDs and the latter the
 *   higher. To minimize computational cost as well as the possibility of wrong
 *   loop closure registration, we search for loop closures only between the
 *   <a>last</a> nodes of group B and the <a>first</a> nodes of group A. Based
on [2] the
 *   potential loop closure edges are not evaluated individually but rather in
 *   sets. Refer to [2] for insight on the algorithm and to
 *   evaluatePartitionsForLC method for the actual implementation. See below
 *   for images of potential Loop closure edges.
 *
 * <div>
 *   <div>
 *     <img src="graphslam-engine_loop_closing_full.png">
 *     <caption align="bottom">
 *     Loop closing schematic. blue nodes belong to group A and have lower
 *     NodeIDs, while red nodes belong to group B and have higher NodeIDs.
 *     Nodes of both groups A and B belong to the same partition based on their
 *     corresponding 2DRangeScan observations.
 *     </caption>
 *   </div>
 *   <div>
 *     <img src="graphslam-engine_loop_closing_consistency_element.png">
 *     <caption align="bottom">
 *     Rigid body transformation of hypotheses and two corresponding
 *     dijkstra links
 *     </caption>
 *   </div>
 * </div>
 *
 * \note
 * Olson uses the following formula for evaluating the pairwise
 * consistency between two hypotheses i,j:
 * <br><center> \f$ A_{i,j} = e^{T \Sigma_T^{-1} T^T} \f$ </center>
 *
 * \note
 * Where:
 * - T is the rigid body transformation using the two hypotheses and the two
 * Dijkstra Links connecting the nodes that form the hypotheses
 * - \f$ \Sigma_T \f$ is the covariance matrix of the aforementioned rigid
 * body transformation
 *
 * \note
 * However this formula is <a>inconsistent with the rest of the paper
 * explanations and mathematical formulas </a>:
 * - The author states that:
 *   \verbatim
"This quantity is proportional to the probability density that the rigid-body
transformation is the identity matrix (i.e., T = [0 0 0])"
\endverbatim
 *   This is \a inconsistent with the given formula. Suppose that a
 *   wrong loop closure is included in the \f$ A_{i,j} \f$, therefore the
 *   pairwise-consistency element should have a low value. For this to hold
 *   true the exponent of the consistency element shoud be small and
 *   neglecting the covariance matrix of rigid-body transformation  (e.g. unit
 *   covariance matrix), \f$ T T^T \f$ should be small.
 *   When a wrong loop closure is evaluated the aforementioned quantity
 *   increases since the hypotheses do not form a correct loop. Therefore the
 *   worse the rigid-body transformation the higher the exponent term,
 *   therefore the higher the consistency element
 * - Author uses the information matrix \f$ \Sigma_T^{-1} \f$ in the
exponential.
 *   However in the optimal case (high certainty of two correct loop closure
 *   hypotheses) information matrix and rigid body transformation vector T
 *   have opposite effects in the exponent term:
 *   - \f$ \text{Correct loop closure} \Rightarrow T \rightarrow [0, 0, 0]
 *   \Rightarrow \text{exponent} \downarrow \f$
 *   - \f$ \text{Correct loop closure} \Rightarrow
 *   \text{diagonal\_terms}(\Sigma_T^{-1}) \uparrow \Rightarrow \text{exponent}
 *   \uparrow \f$
 *
 * \note
 * Based on the previous comments the following formula is used in the decider:
 * <br><center> \f$ A_{i,j} = e^{-T \Sigma_T T^T} \f$ </center>
 *
 * ### .ini Configuration Parameters
 *
 * \htmlinclude graphslam-engine_config_params_preamble.txt
 *
 * - \b class_verbosity
 *   + \a Section       : EdgeRegistrationDeciderParameters
 *   + \a Default value : 1 (mrpt::system::LVL_INFO)
 *   + \a Required      : FALSE
 *
 * - \b use_scan_matching
 *   + \a Section       : EdgeRegistrationDeciderParameters
 *   + \a Default value : TRUE
 *   + \a Required      : FALSE
 *   + \a Description   :Indicates whether the decider uses scan matching
 *   between the current and previous laser scans to correct the robot
 *   trajectory.
 *
 * - \b visualize_laser_scans
 *   + \a Section       : VisualizationParameters
 *   + \a Default value : 10
 *   + \a Required      : FALSE
 *
 * - \b LC_min_nodeid_diff
 *  + \a Section       : GeneralConfiguration
 *  + \a Default value : 30
 *  + \a Required      : FALSE
 *  + \a Description   : Minimum NodeID difference for an edge to be considered
 *  a loop closure.
 *
 * - \b LC_min_remote_nodes
 *   + \a Section       : EdgeRegistrationDeciderParameters
 *   + \a Default value : 3
 *   + \a Required      : FALSE
 *   + \a Description   : Number of remote nodes that must exist in order for
 *   the Loop Closure procedure to commence
 *
 * - \b LC_eigenvalues_ratio_thresh
 *   + \a Section       : EdgeRegistrationDeciderParameters
 *   + \a Default value : 2
 *   + \a Required      : FALSE
 *   + \a Description   : Minimum ratio of the two dominant eigenvalues for a
 *   loop closing hypotheses set to be considered valid
 *
 * - \b LC_check_curr_partition_only
 *   + \a Section       : EdgeRegistrationDeciderParameters
 *   + \a Default value : TRUE
 *   + \a Required      : FALSE
 *   + \a Description   : Boolean flag indicating whether to check for loop
 *   closures only in the current node's partition
 *
 * - \b visualize_map_partitions
 *   + \a Section       : VisualizationParameters
 *   + \a Default value : TRUE
 *   + \a Required      : FALSE
  *
 * \note Class contains an instance of the
 * mrpt::slam::CIncrementalMapPartitioner class and it parses the configuration
 * parameters of the latter from the "EdgeRegistrationDeciderParameters"
 * section. Refer to mrpt::slam::CIncrementalMapPartitioner documentation for
 * its list of configuration parameters
 *
 * \sa mrpt::slam::CIncrementalMapPartitioner
 * \ingroup mrpt_graphslam_grp
 */
template <class GRAPH_T = typename mrpt::graphs::CNetworkOfPoses2DInf>
class CLoopCloserERD : public virtual mrpt::graphslam::deciders::
						   CRangeScanEdgeRegistrationDecider<GRAPH_T>
{
   public:
	/**\brief Edge Registration Decider */
	using parent_t = CRangeScanEdgeRegistrationDecider<GRAPH_T>;

	using constraint_t = typename GRAPH_T::constraint_t;
	using pose_t = typename GRAPH_T::constraint_t::type_value;
	using global_pose_t = typename GRAPH_T::global_pose_t;
	using decider_t = CLoopCloserERD<GRAPH_T>; /**< self type */
	using range_ops_t = typename parent_t::range_ops_t;
	using nodes_to_scans2D_t = typename parent_t::nodes_to_scans2D_t;
	using partitions_t = std::vector<std::vector<uint32_t>>;
	using edges_citerator = typename GRAPH_T::edges_map_t::const_iterator;
	using edges_iterator = typename GRAPH_T::edges_map_t::iterator;
	using hypot_t = typename mrpt::graphs::detail::THypothesis<GRAPH_T>;
	using hypots_t = std::vector<hypot_t>;
	using hypotsp_t = std::vector<hypot_t*>;
	using hypotsp_to_consist_t =
		std::map<std::pair<hypot_t*, hypot_t*>, double>;
	using path_t = mrpt::graphslam::TUncertaintyPath<GRAPH_T>;
	using paths_t = std::vector<path_t>;
	using node_props_t = mrpt::graphslam::detail::TNodeProps<GRAPH_T>;
	/**\}*/

	CLoopCloserERD();
	~CLoopCloserERD() override;

	bool updateState(
		mrpt::obs::CActionCollection::Ptr action,
		mrpt::obs::CSensoryFrame::Ptr observations,
		mrpt::obs::CObservation::Ptr observation) override;

	void setWindowManagerPtr(
		mrpt::graphslam::CWindowManager* win_manager) override;
	void notifyOfWindowEvents(
		const std::map<std::string, bool>& events_occurred) override;
	void getEdgesStats(
		std::map<std::string, int>* edge_types_to_num) const override;

	void initializeVisuals() override;
	void updateVisuals() override;
	void loadParams(const std::string& source_fname) override;
	void printParams() const override;

	void getDescriptiveReport(std::string* report_str) const override;
	void getCurrentPartitions(partitions_t& partitions_out) const;
	const partitions_t& getCurrentPartitions() const;
	/**\brief Return the minimum number of nodes that should exist in the graph
	 * prior to running Dijkstra
	 */
	size_t getDijkstraExecutionThresh() const
	{
		return m_dijkstra_node_count_thresh;
	}
	void setDijkstraExecutionThresh(size_t new_thresh)
	{
		m_dijkstra_node_count_thresh = new_thresh;
	}

	/**\name Helper structs */
	/**\{ */

	/**
	 * \brief Struct for passing additional parameters to the getICPEdge call
	 *
	 * Handy for overriding the search to the \a GRAPH_T::nodes map or the
	 * search for the node's LaserScan
	 */
	struct TGetICPEdgeAdParams
	{
		using self_t = TGetICPEdgeAdParams;

		node_props_t from_params; /**< Ad. params for the from_node */
		node_props_t to_params; /**< Ad. params for the to_node */
		pose_t init_estim; /**< Initial ICP estimation */

		void getAsString(std::string* str) const
		{
			str->clear();
			*str += mrpt::format(
				"from_params: %s", from_params.getAsString().c_str());
			*str +=
				mrpt::format("to_params: %s", to_params.getAsString().c_str());
			*str +=
				mrpt::format("init_estim: %s\n", init_estim.asString().c_str());
		}
		std::string getAsString() const
		{
			std::string str;
			this->getAsString(&str);
			return str;
		}
		friend std::ostream& operator<<(std::ostream& o, const self_t& params)
		{
			o << params.getAsString() << endl;
			return o;
		}
	};
	/**\brief Struct for passing additional parameters to the
	 * generateHypotsPool call
	 */
	struct TGenerateHypotsPoolAdParams
	{
		using group_t = std::map<mrpt::graphs::TNodeID, node_props_t>;

		/**\brief  Ad. params for groupA */
		group_t groupA_params;
		/**\brief  Ad. params for groupB */
		group_t groupB_params;
	};
	/**\} */

	/**\brief Generate the hypothesis pool for all the inter-group constraints
	 * between two groups of nodes.
	 *
	 * \param[in] groupA First group to be tested
	 * \param[in] groupB Second group to be tested
	 * \param[out] generated_hypots Pool of generated hypothesis. Hypotheses
	 * are generated in the heap, so the caller is responsible of afterwards
	 * calling \a delete.
	 */
	void generateHypotsPool(
		const std::vector<uint32_t>& groupA,
		const std::vector<uint32_t>& groupB, hypotsp_t* generated_hypots,
		const TGenerateHypotsPoolAdParams* ad_params = nullptr);
	/**\brief Compute the pair-wise consistencies Matrix.
	 *
	 * \param[in] groupA First group to be used
	 * \param[in] groupB Second group to be used
	 * \param[in] hypots_pool Pool of hypothesis that has been generated
	 * between the two groups
	 * \pram[out] consist_matrix Pointer to Pair-wise consistencies matrix that
	 * is to be filled

	 * \param[in] groupA_opt_paths Pointer to vector of optimal paths that can
	 * be used instead of making queries to the m_node_optimal_paths class
	 * vector. See corresponding argument in generatePWConsistencyElement
	 * method
	 * \param[in] groupB_opt_paths
	 *
	 * \sa generatePWConsistencyElement
	 * \sa evalPWConsistenciesMatrix
	 */
	void generatePWConsistenciesMatrix(
		const std::vector<uint32_t>& groupA,
		const std::vector<uint32_t>& groupB, const hypotsp_t& hypots_pool,
		mrpt::math::CMatrixDouble* consist_matrix,
		const paths_t* groupA_opt_paths = nullptr,
		const paths_t* groupB_opt_paths = nullptr);
	/**\brief Evalute the consistencies matrix, fill the valid hypotheses
	 *
	 * Call to this method should be made right after generating the
	 * consistencies matrix using the generatePWConsistenciesMatrix method
	 *
	 * \sa generatePWConsistenciesMatrix
	 */
	void evalPWConsistenciesMatrix(
		const mrpt::math::CMatrixDouble& consist_matrix,
		const hypotsp_t& hypots_pool, hypotsp_t* valid_hypots);
	// Public variables
	// ////////////////////////////
   protected:
	// protected functions
	//////////////////////////////////////////////////////////////

	/**\brief Fill the TNodeProps instance using the parameters from the map
	 *
	 * \param[in] nodeID ID of node corresponding to the TNodeProps struct that
	 * is to be filled
	 * \param[in] group_params Map of TNodeID to corresponding TNodeProps
	 * instance.
	 * \param[out] node_props Pointer to the TNodeProps struct to be filled.
	 *
	 *
	 * \return True if operation was successful, false otherwise.
	 */
	bool fillNodePropsFromGroupParams(
		const mrpt::graphs::TNodeID& nodeID,
		const std::map<mrpt::graphs::TNodeID, node_props_t>& group_params,
		node_props_t* node_props);
	/**\brief Fill the pose and LaserScan for the given nodeID.
	 * Pose and LaserScan are either fetched from the TNodeProps struct if it
	 * contains valid data, otherwise from the corresponding class vars
	 *
	 * \return True if operation was successful and pose, scan contain valid
	 * data.
	 */
	bool getPropsOfNodeID(
		const mrpt::graphs::TNodeID& nodeID, global_pose_t* pose,
		mrpt::obs::CObservation2DRangeScan::Ptr& scan,
		const node_props_t* node_props = nullptr) const;

	/**\brief Struct for storing together the parameters needed for ICP
	 * matching, laser scans visualization etc.
	 */
	struct TLaserParams : public mrpt::config::CLoadableOptions
	{
	   public:
		TLaserParams();
		~TLaserParams() override;

		void loadFromConfigFile(
			const mrpt::config::CConfigFileBase& source,
			const std::string& section) override;
		void dumpToTextStream(std::ostream& out) const override;
		mrpt::slam::CICP icp;
		/**\brief How many nodes back to check ICP against?
		 */
		int prev_nodes_for_ICP;

		/** see Constructor for initialization */
		const mrpt::img::TColor laser_scans_color =
			mrpt::img::TColor(0, 20, 255);
		bool visualize_laser_scans;
		// keystroke to be used by the user to toggle the LaserScans from
		// the CDisplayWindow
		std::string keystroke_laser_scans = "l";

		/**\brief Indicate whethet to use scan-matching at all during
		 * graphSLAM [on by default].
		 *
		 * \warning It is strongly recomended that the user does not set this
		 * to false (via the .ini file). graphSLAM may diverge significantly if
		 * no scan-matching is not used.
		 */
		bool use_scan_matching;
		bool has_read_config = false;
		;
		/**\brief Keep track of the mahalanobis distance between the initial
		 * pose
		 * difference and the suggested new edge for the pairs of checked
		 * nodes.
		 */
		TSlidingWindow mahal_distance_ICP_odom_win;
		/**\brief Keep track of ICP Goodness values for ICP between nearby
		 * nodes and adapt the Goodness threshold based on the median of the
		 * recorded Goodness values.
		 */
		TSlidingWindow goodness_threshold_win;
	};

	/**\brief Struct for storing together the loop-closing related parameters.
	 */
	struct TLoopClosureParams : public mrpt::config::CLoadableOptions
	{
	   public:
		TLoopClosureParams();
		~TLoopClosureParams() override;

		void loadFromConfigFile(
			const mrpt::config::CConfigFileBase& source,
			const std::string& section) override;
		void dumpToTextStream(std::ostream& out) const override;
		/**\brief flag indicating whether to check only the partition of the
		 * last
		 * registered node for potential loop closures
		 */
		bool LC_check_curr_partition_only;
		/**\brief nodeID difference for detecting potential loop closure in a
		 * partition.
		 *
		 * If this difference is surpassed then the partition should be
		 * investigated for loop closures using Olson's strategy.
		 */
		size_t LC_min_nodeid_diff;
		/**\brief Eigenvalues ratio for accepting/rejecting a hypothesis set.
		 *
		 * By default this is set to 2.
		 */
		double LC_eigenvalues_ratio_thresh;
		/**\brief how many remote nodes (large nodID difference should there be
		 * before I consider the potential loop closure.
		 */
		int LC_min_remote_nodes;
		/**\brief Full partition of map only afer X new nodes have been
		 * registered
		 */
		int full_partition_per_nodes;
		bool visualize_map_partitions;
		std::string keystroke_map_partitions;

		double offset_y_map_partitions;
		int text_index_map_partitions;

		// map partitioning  - visualization window parameters
		const double balloon_elevation{3};
		const double balloon_radius{0.5};
		const mrpt::img::TColor balloon_std_color;
		const mrpt::img::TColor balloon_curr_color;
		const mrpt::img::TColor connecting_lines_color;

		bool has_read_config{false};
	};
	TLaserParams m_laser_params;
	TLoopClosureParams m_lc_params;

	/**brief Compare the suggested ICP edge against the initial node
	 * difference.
	 *
	 * If this difference is significantly larger than the rest of of the
	 * recorded mahalanobis distances, reject the suggested ICP edge.
	 *
	 * \return True if suggested ICP edge is accepted
	 * \note Method updates the Mahalanobis Distance TSlidingWindow which
	 * keep track of the recorded mahalanobis distance values.
	 * \sa getICPEdge
	 */
	bool mahalanobisDistanceOdometryToICPEdge(
		const mrpt::graphs::TNodeID& from, const mrpt::graphs::TNodeID& to,
		const constraint_t& rel_edge);
	/**\brief Wrapper around the registerNewEdge method which accepts a
	 * THypothesis object instead.
	 */
	void registerHypothesis(const hypot_t& h);
	void registerNewEdge(
		const mrpt::graphs::TNodeID& from, const mrpt::graphs::TNodeID& to,
		const constraint_t& rel_edge) override;
	/**\brief Fetch a list of nodes with regards prior to the given nodeID for
	 * which to try and add scan matching edges
	 *
	 * \sa addScanMatchingEdges
	 */
	virtual void fetchNodeIDsForScanMatching(
		const mrpt::graphs::TNodeID& curr_nodeID,
		std::set<mrpt::graphs::TNodeID>* nodes_set);
	/**\brief Addd ICP constraints from X previous nodeIDs up to the given
	 * nodeID.
	 *
	 * X is set by the user in the .ini configuration file (see
	 * TLaserParams::prev_nodes_for_ICP)
	 *
	 * \sa fetchNodeIDsForScanMatching
	 */
	virtual void addScanMatchingEdges(const mrpt::graphs::TNodeID& curr_nodeID);
	void initLaserScansVisualization();
	void updateLaserScansVisualization();
	/**\brief togle the LaserScans visualization on and off
	 */
	void toggleLaserScansVisualization();
	void dumpVisibilityErrorMsg(
		std::string viz_flag, int sleep_time = 500 /* ms */);
	/**\brief Split the currently registered graph nodes into partitions.  */
	void updateMapPartitions(
		bool full_update = false, bool is_first_time_node_reg = false);
	/**\brief Initialize the visualization of the map partition objects. */
	void initMapPartitionsVisualization();
	/**\brief Update the map partitions visualization. */
	void updateMapPartitionsVisualization();
	/**\brief Toggle the map partitions visualization objects.
	 *
	 * To be called  upon relevant keystroke press by the user (see \b
	 * TLoopClosureParams::keystroke_map_partitions)
	 */
	void toggleMapPartitionsVisualization();
	void initCurrCovarianceVisualization();
	void updateCurrCovarianceVisualization();
	/**\brief Compute the Centroid of a group of a vector of node positions.
	 *
	 * \param[in] nodes_list List of node IDs whose positions are taken into
	 * account
	 * \param[out] centroid_coords Contains the Centroid coordinates as a pair
	 * [x,y]
	 *
	 * \note Method is used during the visualization of the map partitions.
	 */
	void computeCentroidOfNodesVector(
		const std::vector<uint32_t>& nodes_list,
		std::pair<double, double>* centroid_coords) const;
	/**\brief Check the registered so far partitions for potential loop
	 * closures.
	 *
	 * Practically checks whether there exist nodes in a single partition whose
	 * distance surpasses the minimum loop closure nodeID distance. The latter
	 * is read from a .ini external file, thus specified by the user (see \b
	 * TLoopClosureParams.LC_min_nodeid_diff.
	 *
	 * \sa evaluatePartitionsForLC
	 */
	void checkPartitionsForLC(partitions_t* partitions_for_LC);
	/**\brief Evaluate the given partitions for loop closures.
	 *
	 * Call this method when you have identified potential loop closures - e.g.
	 * far away nodes in the same partitions - and you want to evaluate the
	 * potential hypotheses in the group. Comprises the main function that
	 * tests potential loop closures in <b>partitions of nodes</b>
	 *
	 * \sa checkPartitionsForLC
	 */
	void evaluatePartitionsForLC(const partitions_t& partitions);

	bool computeDominantEigenVector(
		const mrpt::math::CMatrixDouble& consist_matrix,
		mrpt::math::dynamic_vector<double>* eigvec,
		bool use_power_method = false);
	/**\brief Return the pair-wise consistency between the observations of the
	 * given nodes.
	 *
	 * For the tranformation matrix of the loop use the following edges
	 * - a1=>a2 (Dijkstra Link)
	 * - a2=>b1 (hypothesis - ICP edge)
	 * - b1=>b2 (Dijkstra Link)
	 * - b2=>a1 (hypothesis - ICP edge)
	 *
	 * Given the transformation vector \f$ (x,y,\phi)\f$ of the above
	 * composition (e.g. T) the
	 * pairwise consistency element would then be:
	 * <br><center> \f$ A_{i,j} = e^{-T \Sigma_T T^T} \f$ </center>
	 *
	 * \param[in] hypots Hypothesis corresponding to the potential inter-group
	 * constraints
	 * \param[in] opt_paths Vector of optimal paths that can be used instead of
	 * making queries to the m_node_optimal_paths class vector. See
	 * corresponding argument in generatePWConsistenciesMatrix method
	 * - 1st element \rightarrow a1->a2 path
	 * - 2nd element \rightarrow b1->b2 path
	 *
	 * \return Pairwise consistency eleement of the composition of
	 * transformations
	 *
	 * \sa generatePWConsistenciesMatrix
	 */
	double generatePWConsistencyElement(
		const mrpt::graphs::TNodeID& a1, const mrpt::graphs::TNodeID& a2,
		const mrpt::graphs::TNodeID& b1, const mrpt::graphs::TNodeID& b2,
		const hypotsp_t& hypots, const paths_t* opt_paths = nullptr);
	/**\brief Given a vector of THypothesis objects, find the one that
	 * has the given start and end nodes.
	 *
	 * \note If multiple hypothesis between the same start and end node exist,
	 * only the first one is returned.
	 *
	 * \param[in] vec_hypots Vector of hypothesis to check
	 * \param[in] from Starting Node for hypothesis
	 * \param[in] to Ending Node for hypothesis
	 * \param[in] throw_exc If true and hypothesis is not found, <b>throw a
	 * HypothesisNotFoundException</b>
	 *
	 * \return Pointer to the found hypothesis if that is found, otherwise
	 * nullptr.
	 *
	 */
	static hypot_t* findHypotByEnds(
		const hypotsp_t& vec_hypots, const mrpt::graphs::TNodeID& from,
		const mrpt::graphs::TNodeID& to, bool throw_exc = true);
	/**\brief Given a vector of TUncertaintyPath objects, find the one that has
	 * the given source and destination nodeIDs.
	 *
	 * \note If multiple paths between the same start and end node exist,
	 * only the first one is returned.
	 *
	 * \return nullptr if a path with the given source and destination NodeIDs
	 * is not found, otherwise a pointer to the matching TUncertaintyPath.
	 *
	 * \exception std::runtime_error if path was not found and throw_exc is set
	 * to true
	 */
	static const path_t* findPathByEnds(
		const paths_t& vec_paths, const mrpt::graphs::TNodeID& src,
		const mrpt::graphs::TNodeID& dst, bool throw_exc = true);
	/**\brief Given a vector of THypothesis objects, find the one that
	 * has the given ID.
	 *
	 * \note If multiple hypothesis with the same ID exist, only the first one
	 * is returned.
	 *
	 * \param[in] vec_hypots Vector of hypothesis to check
	 * \param[in] id, ID of the hypothesis to be returned
	 * \param[in] throw_exc If true and hypothesis is not found, <b>throw a
	 * HypothesisNotFoundException</b>
	 *
	 * \return Pointer to the hypothesis with the given ID if that is found,
	 * otherwies nullptr.
	 */
	static hypot_t* findHypotByID(
		const hypotsp_t& vec_hypots, const size_t& id, bool throw_exc = true);
	/**\brief Get the ICP Edge between the provided nodes.
	 *
	 * Handy for not having to manually fetch the laser scans, as the method
	 * takes care of this.
	 *
	 * \param[out] icp_info Struct that will be filled with the results of the
	 * ICP operation
	 *
	 * \param[in] ad_params Pointer to additional parameters in the getICPEdge
	 * call
	 *
	 * \return True if operation was successful, false otherwise (e.g. if the
	 * either of the nodes' CObservation2DRangeScan object does not contain
	 * valid data.
	 */
	virtual bool getICPEdge(
		const mrpt::graphs::TNodeID& from, const mrpt::graphs::TNodeID& to,
		constraint_t* rel_edge,
		mrpt::slam::CICP::TReturnInfo* icp_info = nullptr,
		const TGetICPEdgeAdParams* ad_params = nullptr);
	/**\brief compute the minimum uncertainty of each node position with
	 * regards to the graph root.
	 *
	 * \param[in] starting_node Node from which I start the Dijkstra projection
	 * algorithm
	 * \param[in] ending_node Specify the nodeID whose uncertainty wrt the
	 * starting_node, we are interested in computing. If given, method
	 * execution ends when this path is computed.
	 */
	void execDijkstraProjection(
		mrpt::graphs::TNodeID starting_node = 0,
		mrpt::graphs::TNodeID ending_node = INVALID_NODEID);
	/**\brief Given two nodeIDs compute and return the path connecting them.
	 *
	 * Method takes care of multiple edges, as well as edges with 0 covariance
	 * matrices
	 */
	void getMinUncertaintyPath(
		const mrpt::graphs::TNodeID from, const mrpt::graphs::TNodeID to,
		path_t* path) const;
	/**\brief Find the minimum uncertainty path from te given pool of
	 * TUncertaintyPath instances.
	 *
	 * Removes (and returns) the found path from the pool.
	 *
	 * \return Minimum uncertainty path from the pool provided
	 */
	typename mrpt::graphslam::TUncertaintyPath<GRAPH_T>* popMinUncertaintyPath(
		std::set<path_t*>* pool_of_paths) const;
	/**\brief  Append the paths starting from the current node.
	 *
	 * \param[in] pool_of_paths Paths that are currently registered
	 * \param[in] curr_path Path that I am currently traversing. This path is
	 * already removed from \a pool_of_paths
	 * \param[in] neighbors std::set of neighboring nodes to the last node of
	 * the current path
	 */
	void addToPaths(
		std::set<path_t*>* pool_of_paths, const path_t& curr_path,
		const std::set<mrpt::graphs::TNodeID>& neibors) const;
	/**\brief Query for the optimal path of a nodeID.
	 *
	 * Method handles calls to out-of-bounds nodes as well as nodes whose paths
	 * have not yet been computed.
	 *
	 * \param[in] node nodeID for which hte path is going to be returned
	 *
	 * \return Optimal path corresponding to the given nodeID or nullptr if the
	 * former is not found.
	 */
	typename mrpt::graphslam::TUncertaintyPath<GRAPH_T>* queryOptimalPath(
		const mrpt::graphs::TNodeID node) const;
	/**\brief Split an existing partition to Groups
	 *
	 *	 Have two groups A, B.
	 *	 - Group A consists of the lower nodeIDs. They correspond to the start
	 *	 of the course
	 *	 - Group B consists of the higher (more recent) nodeIDs. They
	 *	 correspond to the end of the course find where to split the current
	 *	 partition
	 *
	 *	 \note Method is used in single-robot graphSLAM for spliting a
	 *	 partition of nodes to lower and higher node IDs
	 *
	 *	 \param[in] partition Partition to be split.
	 *	 \param[out] groupA First group of nodes.
	 *	 \param[out] groupB Second group of nodes.
	 *	 \param[in] max_nodes_in_group Max number of nodes that are to exist in
	 *	 each group (Use -1 to disable this threshold).
	 */
	void splitPartitionToGroups(
		std::vector<uint32_t>& partition, std::vector<uint32_t>* groupA,
		std::vector<uint32_t>* groupB, int max_nodes_in_group = 5);
	/**\brief Assign the last recorded 2D Laser scan
	 *
	 * \note Compact way of assigning the last recorded laser scan for both
	 * MRPT rawlog formats.
	 *
	 * Method takes into account the start of graphSLAM proc. when two nodes
	 * are added at the graph at the same time (root + node for 1st constraint)
	 */
	void setLastLaserScan2D(mrpt::obs::CObservation2DRangeScan::Ptr scan);

	/**\brief Instance responsible for partitioning the map */
	mrpt::slam::CIncrementalMapPartitioner m_partitioner;

	bool m_visualize_curr_node_covariance = false;
	const mrpt::img::TColor m_curr_node_covariance_color =
		mrpt::img::TColor(160, 160, 160, 255);
	double m_offset_y_curr_node_covariance;
	int m_text_index_curr_node_covariance;

	/**\brief Keep track of the registered edge types.
	 *
	 * Handy for displaying them in the Visualization window.
	 */
	std::map<std::string, int> m_edge_types_to_nums;
	/**\brief Keep the last laser scan for visualization purposes */
	mrpt::obs::CObservation2DRangeScan::Ptr m_last_laser_scan2D;
	/**\name Partition vectors */
	/**\{ */
	/**\brief Previous partitions vector */
	partitions_t m_last_partitions;
	/**\brief Current partitions vector */
	partitions_t m_curr_partitions;
	/**\} */
	/**\brief Indicate whether the partitions have been updated recently */
	bool m_partitions_full_update = false;
	/**\brief Keep track of the evaluated partitions so they are not checked
	 * again if nothing changed in them.
	 */
	std::map<int, std::vector<uint32_t>> m_partitionID_to_prev_nodes_list;
	/**\brief Map that stores the lowest uncertainty path towards a node.
	 * Starting node depends on the starting node as used in the
	 * execDijkstraProjection method
	 */
	typename std::map<mrpt::graphs::TNodeID, path_t*> m_node_optimal_paths;
	/**\brief Keep track of the first recorded laser scan so that it can be
	 * assigned to the root node when the NRD adds the first *two* nodes to the
	 * graph.
	 */
	mrpt::obs::CObservation2DRangeScan::Ptr m_first_laser_scan;
	/**\brief Track the first node registration occurance
	 *
	 * Handy so that we can assign a measurement to the root node as well.
	 */
	bool m_is_first_time_node_reg = true;
	/**\brief Node Count lower bound before executing dijkstra
	 */
	size_t m_dijkstra_node_count_thresh = 3;
	/**\brief Factor used for accepting an ICP Constraint as valid.
	 */
	double m_consec_icp_constraint_factor;
	/**\brief Factor used for accepting an ICP Constraint in the loop closure
	 * proc.
	 */
	double m_lc_icp_constraint_factor;
};
}  // namespace mrpt::graphslam::deciders
#include "CLoopCloserERD_impl.h"
