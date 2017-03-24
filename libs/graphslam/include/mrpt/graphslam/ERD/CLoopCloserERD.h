/* +---------------------------------------------------------------------------+
	 |                     Mobile Robot Programming Toolkit (MRPT)               |
	 |                          http://www.mrpt.org/                             |
	 |                                                                           |
	 | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
	 | See: http://www.mrpt.org/Authors - All rights reserved.                   |
	 | Released under BSD License. See details in http://www.mrpt.org/License    |
	 +---------------------------------------------------------------------------+ */

#ifndef CLOOPCLOSERERD_H
#define CLOOPCLOSERERD_H

#include <mrpt/math/CMatrix.h>
#include <mrpt/math/utils.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/CConfigFileBase.h>
#include <mrpt/utils/types_simple.h>
#include <mrpt/utils/TColor.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/CSphere.h>
#include <mrpt/opengl/CEllipsoid.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/slam/CIncrementalMapPartitioner.h>
#include <mrpt/slam/CICP.h>
#include <mrpt/system/os.h>
#include <mrpt/system/threads.h>
#include <mrpt/math/data_utils.h>

#include <mrpt/graphslam/interfaces/CEdgeRegistrationDecider.h>
#include <mrpt/graphslam/misc/TSlidingWindow.h>
#include <mrpt/graphslam/misc/CRangeScanRegistrationDecider.h>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <iterator>
#include <map>
#include <vector>
#include <string>
#include <set>
#include <sstream>
#include <stdlib.h> // abs

namespace mrpt { namespace graphslam { namespace deciders {

/**\brief Edge Registration Decider scheme specialized in Loop Closing.
 *
 * ## Description
 *
 * Current decider is implemented based on the following papers:
 *
 * - [1] <a
 *   href="http://ieeexplore.ieee.org/xpl/login.jsp?tp=&arnumber=1641810&url=http%3A%2F%2Fieeexplore.ieee.org%2Fxpls%2Fabs_all.jsp%3Farnumber%3D1641810">Consistent
 * Observation Grouping for Generating Metric-Topological Maps that Improves
 * Robot Localization</a> - J. Blanco, J. Gonzalez, J. Antonio Fernandez
 * Madrigal, 2006
 * - [2] <a href="https://april.eecs.umich.edu/pdfs/olson2009ras.pdf">Recognizing
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
 *   contain loop closure edges (these groups contain successive nodes with large
 *   difference in their IDs). These groups are then split into two subgroups
 *   A, B, with the former containing the lower NodeIDs and the latter the
 *   higher. To minimize computational cost as well as the possibility of wrong
 *   loop closure registration, we search for loop closures only between the
 *   <a>last</a> nodes of group B and the <a>first</a> nodes of group A. Based on [2] the
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
 * - Author uses the information matrix \f$ \Sigma_T^{-1} \f$ in the exponential.  
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
 *   + \a Default value : 1 (LVL_INFO)
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
 * \note Since the decider inherits from the CRangeScanRegistrationDecider
 * class, it parses the configuration parameters of the latter as well from the
 * "ICP" section. Refer to the CRangeScanRegistrationDecider documentation for
 * its list of configuration parameters
 *
 * \note Class contains an instance of the
 * mrpt::slam::CIncrementalMapPartitioner class and it parses the configuration
 * parameters of the latter from the "EdgeRegistrationDeciderParameters"
 * section. Refer to mrpt::slam::CIncrementalMapPartitioner documentation for
 * its list of configuration parameters
 *
 * \ingroup mrpt_graphslam_grp
 */
template<class GRAPH_t=typename mrpt::graphs::CNetworkOfPoses2DInf >
class CLoopCloserERD:
	public mrpt::graphslam::deciders::CEdgeRegistrationDecider<GRAPH_t>,
	public mrpt::graphslam::deciders::CRangeScanRegistrationDecider<GRAPH_t>
{
	public:
		/**\brief type of graph constraints */
		typedef typename GRAPH_t::constraint_t constraint_t;
		/**\brief type of underlying poses (2D/3D). */
		typedef typename GRAPH_t::constraint_t::type_value pose_t;
		/**\brief Typedef for accessing methods of the RangeScanRegistrationDecider_t parent class. */
		typedef mrpt::graphslam::deciders::CRangeScanRegistrationDecider<GRAPH_t> range_scanner_t;
		typedef CLoopCloserERD<GRAPH_t> decider_t; /**< self type - Handy typedef */
		/**\brief New typedef for splitting the nodes into groups */
		typedef std::vector<mrpt::vector_uint> partitions_t;
		typedef std::map<mrpt::utils::TNodeID, mrpt::obs::CObservation2DRangeScanPtr> nodes_to_scans2D_t;
		typedef typename GRAPH_t::edges_map_t::const_iterator edges_citerator;
		typedef typename GRAPH_t::edges_map_t::iterator edges_iterator;

		// Public methods
		//////////////////////////////////////////////////////////////
		CLoopCloserERD();
		~CLoopCloserERD();

		bool updateState(
				mrpt::obs::CActionCollectionPtr action,
				mrpt::obs::CSensoryFramePtr observations,
				mrpt::obs::CObservationPtr observation );

		void setGraphPtr(GRAPH_t* graph);
		void setWindowManagerPtr(mrpt::graphslam::CWindowManager* win_manager);
		void notifyOfWindowEvents(
				const std::map<std::string, bool>& events_occurred);
		void getEdgesStats(
				std::map<std::string, int>* edge_types_to_num) const;

		void initializeVisuals();
		void updateVisuals();
		bool justInsertedLoopClosure() const;
		void loadParams(const std::string& source_fname);
		void printParams() const;


		void getDescriptiveReport(std::string* report_str) const;

		// Public variables
		// ////////////////////////////

	private:
		// Private functions
		//////////////////////////////////////////////////////////////

		/**\brief Struct for storing together the parameters needed for ICP
		 * matching, laser scans visualization etc.
		 */
		struct TLaserParams: public mrpt::utils::CLoadableOptions {
			public:
				TLaserParams();
				~TLaserParams();

				void loadFromConfigFile(
						const mrpt::utils::CConfigFileBase &source,
						const std::string &section);
				void 	dumpToTextStream(mrpt::utils::CStream &out) const;

				mrpt::slam::CICP icp;
				// threshold for accepting an ICP constraint in the graph
				size_t prev_nodes_for_ICP; // how many nodes back to check ICP against?

 				/** see Constructor for initialization */
				const mrpt::utils::TColor laser_scans_color;
				bool visualize_laser_scans;
				// keystroke to be used by the user to toggle the LaserScans from
				// the CDisplayWindow
				std::string keystroke_laser_scans;

				/**\brief Indicate whethet to use scan-matching at all during
				 * graphSLAM [on by default].
				 *
				 * \warning It is strongly recomended that the user does not set this
				 * to false (via the .ini file). graphSLAM may diverge significantly if
				 * no scan-matching is not used.
				 */
				bool use_scan_matching;
				bool has_read_config;
				/**\brief Keep track of the mahalanobis distance between the initial pose
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
		struct TLoopClosureParams: public mrpt::utils::CLoadableOptions {
			public:
				TLoopClosureParams();
				~TLoopClosureParams();

				void loadFromConfigFile(
						const mrpt::utils::CConfigFileBase &source,
						const std::string &section);
				void 	dumpToTextStream(mrpt::utils::CStream &out) const;

				/**\brief flag indicating whether to check only the partition of the last
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
				const double balloon_elevation;
				const double balloon_radius;
				const mrpt::utils::TColor balloon_std_color;
				const mrpt::utils::TColor balloon_curr_color;
				const mrpt::utils::TColor connecting_lines_color;

				bool has_read_config;


		};
		TLaserParams m_laser_params;
		TLoopClosureParams m_lc_params;

		/**\brief Holds the data of an information path.
		 *
		 * Instances of this struct are used during the Dijkstra projection method
		 */
		struct TPath : public mrpt::utils::CLoadableOptions {

			// methods
			// ////////////////////////////
			TPath();
			TPath(mrpt::utils::TNodeID starting_node);
			~TPath();
			void clear();

			// no need to load anything..
			void loadFromConfigFile(
					const mrpt::utils::CConfigFileBase &source,
					const std::string &section);
			void 	dumpToTextStream(mrpt::utils::CStream &out) const;
			std::string getAsString() const;
			void getAsString(std::string* str) const;

			mrpt::utils::TNodeID getSource() const;
			mrpt::utils::TNodeID getDestination() const;

			double getDeterminant();

			/**\brief Test if the current path has a lower uncertainty than the other
			 * path.
			 *
			 * \return True if the current path does have a lower uncertainty
			 */
			bool hasLowerUncertaintyThan(const TPath& other) const;

			/**\brief add a new link in the current path.
			 *
			 * Add the node that the path traverses and the information matrix of
			 * the extra link
			 */
			void addToPath(mrpt::utils::TNodeID node, constraint_t edge);

			/**brief Test weather the constraints are of type CPosePDFGaussianInf.*/
			bool isGaussianInfType() const;
			/**brief Test weather the constraints are of type CPosePDFGaussian.  */
			bool isGaussianType() const;

			TPath& operator+=(const TPath& other);
			// results...
			bool operator==(const TPath& other);
			bool operator!=(const TPath& other);

			// members
			// ////////////////////////////

			/**\brief Nodes that the path comprises of.
			 * Nodes in the path are added to the end of the vector
			 */
			std::vector<mrpt::utils::TNodeID> nodes_traversed;
			/**\brief Current path position + related covariance */
			constraint_t curr_pose_pdf;

			bool determinant_updated;
			double determinant_cached;

		};
		struct THypothesis {
			THypothesis() {is_valid = true; }
			~THypothesis() { }
			std::string getAsString(bool oneline=true) const;
			void getAsString(std::string* str, bool oneline=true) const;

			int id;
			mrpt::utils::TNodeID from;
			mrpt::utils::TNodeID to;

			constraint_t edge;
			double goodness;
			bool is_valid;

		};

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
		bool mahalanobisDistanceOdometryToICPEdge(const mrpt::utils::TNodeID& from,
				const mrpt::utils::TNodeID& to, const constraint_t& rel_edge);

		/**\brief Wrapper around the registerNewEdge method which accepts a
		 * THypothesis object instead.
		 */
		void registerHypothesis(const THypothesis& h);

		/** \brief Initialization function to be called from the various
		 * constructors.
		 */
		void initCLoopCloserERD();
		void registerNewEdge(
				const mrpt::utils::TNodeID& from,
				const mrpt::utils::TNodeID& to,
				const constraint_t& rel_edge );
		/**\brief Addd ICP constraints from X previous nodeIDs up to the given
		 * nodeID.
		 *
		 * X is set by the user in the .ini configuration file (see
		 * TLaserParams::prev_nodes_for_ICP)
		 */
		void addScanMatchingEdges(mrpt::utils::TNodeID curr_nodeID);

		void initLaserScansVisualization();
		void updateLaserScansVisualization();
		/**\brief togle the LaserScans visualization on and off
		 */
		void toggleLaserScansVisualization();
		void dumpVisibilityErrorMsg(std::string viz_flag,
				int sleep_time=500 /* ms */);
		void checkIfInvalidDataset(mrpt::obs::CActionCollectionPtr action,
				mrpt::obs::CSensoryFramePtr observations,
				mrpt::obs::CObservationPtr observation );
		/**\brief Split the currently registered graph nodes into partitions.  */
		void updateMapPartitions(bool full_update=false);
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

		void computeCentroidOfNodesVector(const vector_uint& nodes_list,
				std::pair<double, double>* centroid_coords) const;

		template<class T>
		static void printVectorOfVectors(const T& t);
		template<class T>
		static void printVector(const T& t);
		template<class T>
		static std::string getVectorAsString(const T& t);


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
		 * potential hypotheses in the group
		 *
		 * \sa checkPartitionsForLC
		 */
		void evaluatePartitionsForLC(const partitions_t& partitions);
		bool computeDominantEigenVector(const mrpt::math::CMatrixDouble& consist_matrix,
				mrpt::math::dynamic_vector<double>* eigvec,
				bool use_power_method=false);
		/**\brief Return the pair-wise consistency between the observations of the
		 * given nodes.
		 *
		 * For the tranformation matrix of the loop use the following edges
		 * - a1=>a2 (Dijkstra Link)
		 * - a2=>b1 (hypothesis - ICP edge)
		 * - b1=>b2 (Dijkstra Link)
		 * - b2=>a1 (hypothesis - ICP edge)
		 *
		 * Given the transformation vector \f$ (x,y,\phi)\f$ of the above composition (e.g. T) the
		 * pairwise consistency element would then be: 
 		 * <br><center> \f$ A_{i,j} = e^{-T \Sigma_T T^T} \f$ </center>
		 *
		 */
		double generatePWConsistencyElement(
				const mrpt::utils::TNodeID& a1,
				const mrpt::utils::TNodeID& a2,
				const mrpt::utils::TNodeID& b1,
				const mrpt::utils::TNodeID& b2,
				const std::map<std::pair<mrpt::utils::TNodeID, mrpt::utils::TNodeID>,
					THypothesis*>& hypots_map);
		
		/** Get the ICP Edge between the provided nodes.
		 *
		 * Handy for not having to manually fetch the laser scans, as the method
		 * takes care of this.
		 *
		 * \return True if operation was successful, false otherwise (e.g. if the
		 * either of the nodes' CObservation2DRangeScan object does not contain
		 * valid data.
		 */
		bool getICPEdge(
				const mrpt::utils::TNodeID& from,
				const mrpt::utils::TNodeID& to,
				constraint_t* rel_edge,
				mrpt::slam::CICP::TReturnInfo* icp_info=NULL);

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
				mrpt::utils::TNodeID starting_node=0,
				mrpt::utils::TNodeID ending_node=INVALID_NODEID);
		/**\brief Given two nodeIDs compute and return the path connecting them.
		 *
		 * Method takes care of multiple edges, as well as edges with 0 covariance
		 * matrices
		 */
		void getMinUncertaintyPath(
				const mrpt::utils::TNodeID from,
				const mrpt::utils::TNodeID to,
				TPath* path) const;
		/**\brief Find the minimum uncertainty path from te given pool of TPath
		 * instances.
		 *
		 * Removes (and returns) the found path from the pool.
		 *
		 * \return Minimum uncertainty path from the pool provided
		 */
		TPath* popMinUncertaintyPath(std::set<TPath*>* pool_of_paths) const;
		/**\brief  Append the paths starting from the current node.
		 *
		 * \param[in] pool_of_paths Paths that are currently registered
		 * \param[in] curr_path Path that I am currently traversing. This path is
		 * already removed from \a pool_of_paths
		 * \param[in] neighbors std::set of neighboring nodes to the last node of
		 * the current path
		 */
		void addToPaths(std::set<TPath*>* pool_of_paths,
				const TPath& curr_path,
				const std::set<mrpt::utils::TNodeID>& neibors) const;
		/**\brief
		 * Query for the optimal path of a nodeID. 
		 *
		 * Method handles calls to out-of-bounds nodes as well as nodes whose paths
		 * have not yet been computed.
		 *
		 * \param[in] node nodeID for which hte path is going to be returned
		 *
		 * \return Optimal path corresponding to the given nodeID or NULL if the
		 * former is not found.
		 */
		TPath* queryOptimalPath(const mrpt::utils::TNodeID node) const;

		// Private variables
		//////////////////////////////////////////////////////////////
		/**\brief Instance responsible for partitioning the map */
		mrpt::slam::CIncrementalMapPartitioner m_partitioner;

		GRAPH_t* m_graph; /**<\brief Pointer to the graph under construction */
		mrpt::gui::CDisplayWindow3D* m_win;
		mrpt::graphslam::CWindowManager* m_win_manager;
		mrpt::graphslam::CWindowObserver* m_win_observer;

		bool m_initialized_visuals;
		bool m_just_inserted_loop_closure;

		bool m_visualize_curr_node_covariance;
		const mrpt::utils::TColor m_curr_node_covariance_color;
		double m_offset_y_curr_node_covariance;
		int m_text_index_curr_node_covariance;

		/**\brief Keep track of the registered edge types.
		 *
		 * Handy for displaying them in the Visualization window.
		 */
		std::map<std::string, int> m_edge_types_to_nums;
 		/**\brief Keep track of the total number of registered nodes since the last
 		 * time class method was called */
		size_t m_last_total_num_of_nodes;
		/**\brief Surpass this to start adding edges */
		int m_threshold_to_start;

		/**\brief Map for keeping track of the observation recorded at each graph
		 * position
		 */
		nodes_to_scans2D_t  m_nodes_to_laser_scans2D;
		/**\brief Keep the last laser scan for visualization purposes */
		mrpt::obs::CObservation2DRangeScanPtr m_last_laser_scan2D;

		/**\brief Previous partitions vector */
		partitions_t m_last_partitions;
		/**\brief Current partitions vector */
		partitions_t m_curr_partitions;
		/**\brief Indicate whether the partitions have been updated recently */
		bool m_partitions_full_update;

		/**\brief Keep track of the evaluated partitions so they are not checked
		 * again if nothing changed in them.
		 */
		std::map<int, vector_uint> m_partitionID_to_prev_nodes_list;

		// find out if decider is invalid for the given dataset
		bool m_checked_for_usuable_dataset;
		size_t m_consecutive_invalid_format_instances;
		const size_t m_consecutive_invalid_format_instances_thres;
		
		/**\brief Map for holding the information matrix representing the
		 * certanty of each node position
		 */
		std::map<mrpt::utils::TNodeID, TPath*> m_node_optimal_paths;
		mrpt::utils::CTimeLogger m_time_logger; /**<Time logger instance */

		const std::string m_class_name;


};

} } } // end of namespaces


#include "CLoopCloserERD_impl.h"
#endif /* end of include guard: CLOOPCLOSERERD_H */
